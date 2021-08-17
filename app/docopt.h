//
//  docopt.h
//  docopt
//
//  Created by Jared Grubb on 2013-11-03.
//  Copyright (c) 2013 Jared Grubb. All rights reserved.
//

#ifndef docopt__docopt_h_
#define docopt__docopt_h_

#include <cassert>
#include <cstddef>
#include <functional> // std::hash
#include <iosfwd>
#include <iostream>
#include <map>
#include <regex>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#define DOCOPT_INLINE inline
#define DOCOPT_API

namespace docopt {

	/// A generic type to hold the various types that can be produced by docopt.
	///
	/// This type can be one of: {bool, long, string, vector<string>}, or empty.
	struct value {
		/// An empty value
		value() {}

		value(std::string);
		value(std::vector<std::string>);

		explicit value(bool);
		explicit value(long);
		explicit value(int v) : value(static_cast<long>(v)) {}

		~value();
		value(value const&);
		value(value&&) noexcept;
		value& operator=(value const&);
		value& operator=(value&&) noexcept;

		// Test if this object has any contents at all
		explicit operator bool() const { return kind != Kind::Empty; }

		// Test the type contained by this value object
		bool isBool()       const { return kind==Kind::Bool; }
		bool isString()     const { return kind==Kind::String; }
		bool isLong()       const { return kind==Kind::Long; }
		bool isStringList() const { return kind==Kind::StringList; }

		// Throws std::invalid_argument if the type does not match
		bool asBool() const;
		long asLong() const;
		std::string const& asString() const;
		std::vector<std::string> const& asStringList() const;

		size_t hash() const noexcept;

		// equality is based on hash-equality
		friend bool operator==(value const&, value const&);
		friend bool operator!=(value const&, value const&);

	private:
		enum class Kind {
			Empty,
			Bool,
			Long,
			String,
			StringList
		};

		union Variant {
			Variant() {}
			~Variant() {  /* do nothing; will be destroyed by ~value */ }

			bool boolValue;
			long longValue;
			std::string strValue;
			std::vector<std::string> strList;
		};

		static const char* kindAsString(Kind kind) {
			switch (kind) {
				case Kind::Empty: return "empty";
				case Kind::Bool: return "bool";
				case Kind::Long: return "long";
				case Kind::String: return "string";
				case Kind::StringList: return "string-list";
			}
			return "unknown";
		}

		void throwIfNotKind(Kind expected) const {
			if (kind == expected)
				return;

			std::string error = "Illegal cast to ";
			error += kindAsString(expected);
			error += "; type is actually ";
			error += kindAsString(kind);
			throw std::runtime_error(std::move(error));
		}

	private:
		Kind kind = Kind::Empty;
		Variant variant {};
	};

	/// Write out the contents to the ostream
	std::ostream& operator<<(std::ostream&, value const&);
}

namespace std {
	template <>
	struct hash<docopt::value> {
		size_t operator()(docopt::value const& val) const noexcept {
			return val.hash();
		}
	};
}

namespace docopt {
	inline
	value::value(bool v)
	: kind(Kind::Bool)
	{
		variant.boolValue = v;
	}

	inline
	value::value(long v)
	: kind(Kind::Long)
	{
		variant.longValue = v;
	}

	inline
	value::value(std::string v)
	: kind(Kind::String)
	{
		new (&variant.strValue) std::string(std::move(v));
	}

	inline
	value::value(std::vector<std::string> v)
	: kind(Kind::StringList)
	{
		new (&variant.strList) std::vector<std::string>(std::move(v));
	}

	inline
	value::value(value const& other)
	: kind(other.kind)
	{
		switch (kind) {
			case Kind::String:
				new (&variant.strValue) std::string(other.variant.strValue);
				break;

			case Kind::StringList:
				new (&variant.strList) std::vector<std::string>(other.variant.strList);
				break;

			case Kind::Bool:
				variant.boolValue = other.variant.boolValue;
				break;

			case Kind::Long:
				variant.longValue = other.variant.longValue;
				break;

			case Kind::Empty:
			default:
				break;
		}
	}

	inline
	value::value(value&& other) noexcept
	: kind(other.kind)
	{
		switch (kind) {
			case Kind::String:
				new (&variant.strValue) std::string(std::move(other.variant.strValue));
				break;

			case Kind::StringList:
				new (&variant.strList) std::vector<std::string>(std::move(other.variant.strList));
				break;

			case Kind::Bool:
				variant.boolValue = other.variant.boolValue;
				break;

			case Kind::Long:
				variant.longValue = other.variant.longValue;
				break;

			case Kind::Empty:
			default:
				break;
		}
	}

	inline
	value::~value()
	{
		switch (kind) {
			case Kind::String:
				variant.strValue.~basic_string();
				break;

			case Kind::StringList:
				variant.strList.~vector();
				break;

			case Kind::Empty:
			case Kind::Bool:
			case Kind::Long:
			default:
				// trivial dtor
				break;
		}
	}

	inline
	value& value::operator=(value const& other) {
		// make a copy and move from it; way easier.
		return *this = value{other};
	}

	inline
	value& value::operator=(value&& other) noexcept {
		// move of all the types involved is noexcept, so we dont have to worry about
		// these two statements throwing, which gives us a consistency guarantee.
		this->~value();
		new (this) value(std::move(other));

		return *this;
	}

	template <class T>
	void hash_combine(std::size_t& seed, const T& v);

	inline
	size_t value::hash() const noexcept
	{
		switch (kind) {
			case Kind::String:
				return std::hash<std::string>()(variant.strValue);

			case Kind::StringList: {
				size_t seed = std::hash<size_t>()(variant.strList.size());
				for(auto const& str : variant.strList) {
					hash_combine(seed, str);
				}
				return seed;
			}

			case Kind::Bool:
				return std::hash<bool>()(variant.boolValue);

			case Kind::Long:
				return std::hash<long>()(variant.longValue);

			case Kind::Empty:
			default:
				return std::hash<void*>()(nullptr);
		}
	}

	inline
	bool value::asBool() const
	{
		throwIfNotKind(Kind::Bool);
		return variant.boolValue;
	}

	inline
	long value::asLong() const
	{
		// Attempt to convert a string to a long
		if (kind == Kind::String) {
			const std::string& str = variant.strValue;
			std::size_t pos;
			const long ret = stol(str, &pos); // Throws if it can't convert
			if (pos != str.length()) {
				// The string ended in non-digits.
				throw std::runtime_error( str + " contains non-numeric characters.");
			}
			return ret;
		}
		throwIfNotKind(Kind::Long);
		return variant.longValue;
	}

	inline
	std::string const& value::asString() const
	{
		throwIfNotKind(Kind::String);
		return variant.strValue;
	}

	inline
	std::vector<std::string> const& value::asStringList() const
	{
		throwIfNotKind(Kind::StringList);
		return variant.strList;
	}

	inline
	bool operator==(value const& v1, value const& v2)
	{
		if (v1.kind != v2.kind)
			return false;

		switch (v1.kind) {
			case value::Kind::String:
				return v1.variant.strValue==v2.variant.strValue;

			case value::Kind::StringList:
				return v1.variant.strList==v2.variant.strList;

			case value::Kind::Bool:
				return v1.variant.boolValue==v2.variant.boolValue;

			case value::Kind::Long:
				return v1.variant.longValue==v2.variant.longValue;

			case value::Kind::Empty:
			default:
				return true;
		}
	}

	inline
	bool operator!=(value const& v1, value const& v2)
	{
		return !(v1 == v2);
	}

	// Usage string could not be parsed (ie, the developer did something wrong)
	struct DocoptLanguageError : std::runtime_error { using runtime_error::runtime_error; };

	// Arguments passed by user were incorrect (ie, developer was good, user is wrong)
	struct DocoptArgumentError : std::runtime_error { using runtime_error::runtime_error; };

	// Arguments contained '--help' and parsing was aborted early
	struct DocoptExitHelp : std::runtime_error { DocoptExitHelp() : std::runtime_error("Docopt --help argument encountered"){} };

	// Arguments contained '--version' and parsing was aborted early
	struct DocoptExitVersion : std::runtime_error { DocoptExitVersion() : std::runtime_error("Docopt --version argument encountered") {} };

	/// Parse user options from the given option string.
	///
	/// @param doc   The usage string
	/// @param argv  The user-supplied arguments
	/// @param help  Whether to end early if '-h' or '--help' is in the argv
	/// @param version Whether to end early if '--version' is in the argv
	/// @param options_first  Whether options must precede all args (true), or if args and options
	///                can be arbitrarily mixed.
	///
	/// @throws DocoptLanguageError if the doc usage string had errors itself
	/// @throws DocoptExitHelp if 'help' is true and the user has passed the '--help' argument
	/// @throws DocoptExitVersion if 'version' is true and the user has passed the '--version' argument
	/// @throws DocoptArgumentError if the user's argv did not match the usage patterns
	std::map<std::string, value> DOCOPT_API docopt_parse(std::string const& doc,
					    std::vector<std::string> const& argv,
					    bool help = true,
					    bool version = true,
					    bool options_first = false);

	/// Parse user options from the given string, and exit appropriately
	///
	/// Calls 'docopt_parse' and will terminate the program if any of the exceptions above occur:
	///  * DocoptLanguageError - print error and terminate (with exit code -1)
	///  * DocoptExitHelp - print usage string and terminate (with exit code 0)
	///  * DocoptExitVersion - print version and terminate (with exit code 0)
	///  * DocoptArgumentError - print error and usage string and terminate (with exit code -1)
	std::map<std::string, value> DOCOPT_API docopt(std::string const& doc,
					    std::vector<std::string> const& argv,
					    bool help = true,
					    std::string const& version = {},
					    bool options_first = false) noexcept;
}

namespace {
	bool starts_with(std::string const& str, std::string const& prefix)
	{
		if (str.length() < prefix.length())
			return false;
		return std::equal(prefix.begin(), prefix.end(),
				  str.begin());
	}

	std::string trim(std::string&& str,
			 const std::string& whitespace = " \t\n")
	{
		const auto strEnd = str.find_last_not_of(whitespace);
		if (strEnd==std::string::npos)
			return {}; // no content
		str.erase(strEnd+1);

		const auto strBegin = str.find_first_not_of(whitespace);
		str.erase(0, strBegin);

		return std::move(str);
	}

	std::vector<std::string> split(std::string const& str, size_t pos = 0)
	{
		const char* const anySpace = " \t\r\n\v\f";

		std::vector<std::string> ret;
		while (pos != std::string::npos) {
			auto start = str.find_first_not_of(anySpace, pos);
			if (start == std::string::npos) break;

			auto end = str.find_first_of(anySpace, start);
			auto size = end==std::string::npos ? end : end-start;
			ret.emplace_back(str.substr(start, size));

			pos = end;
		}

		return ret;
	}

	std::tuple<std::string, std::string, std::string> partition(std::string str, std::string const& point)
	{
		std::tuple<std::string, std::string, std::string> ret;

		auto i = str.find(point);

		if (i == std::string::npos) {
			// no match: string goes in 0th spot only
		} else {
			std::get<2>(ret) = str.substr(i + point.size());
			std::get<1>(ret) = point;
			str.resize(i);
		}
		std::get<0>(ret) = std::move(str);

		return ret;
	}

	template <typename I>
	std::string join(I iter, I end, std::string const& delim) {
		if (iter==end)
			return {};

		std::string ret = *iter;
		for(++iter; iter!=end; ++iter) {
			ret.append(delim);
			ret.append(*iter);
		}
		return ret;
	}

	std::vector<std::string> regex_split(std::string const& text, std::regex const& re)
	{
		std::vector<std::string> ret;
		for (auto it = std::sregex_token_iterator(text.begin(), text.end(), re, -1);
			it != std::sregex_token_iterator();
			++it) {
			ret.emplace_back(*it);
		}
		return ret;
	}
}

namespace docopt {
	template <class T>
	inline void hash_combine(std::size_t& seed, T const& v)
	{
		// stolen from boost::hash_combine
		std::hash<T> hasher;
		seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
	}
}

namespace docopt {

	class Pattern;
	class LeafPattern;

	using PatternList = std::vector<std::shared_ptr<Pattern>>;

	// Utility to use Pattern types in std hash-containers
	struct PatternHasher {
		template <typename P>
		size_t operator()(std::shared_ptr<P> const& pattern) const {
			return pattern->hash();
		}
		template <typename P>
		size_t operator()(P const* pattern) const {
			return pattern->hash();
		}
		template <typename P>
		size_t operator()(P const& pattern) const {
			return pattern.hash();
		}
	};

	// Utility to use 'hash' as the equality operator as well in std containers
	struct PatternPointerEquality {
		template <typename P1, typename P2>
		bool operator()(std::shared_ptr<P1> const& p1, std::shared_ptr<P2> const& p2) const {
			return p1->hash()==p2->hash();
		}
		template <typename P1, typename P2>
		bool operator()(P1 const* p1, P2 const* p2) const {
			return p1->hash()==p2->hash();
		}
	};

	// A hash-set that uniques by hash value
	using UniquePatternSet = std::unordered_set<std::shared_ptr<Pattern>, PatternHasher, PatternPointerEquality>;


	class Pattern {
	public:
		// flatten out children, stopping descent when the given filter returns 'true'
		virtual std::vector<Pattern*> flat(bool (*filter)(Pattern const*)) = 0;

		// flatten out all children into a list of LeafPattern objects
		virtual void collect_leaves(std::vector<LeafPattern*>&) = 0;

		// flatten out all children into a list of LeafPattern objects
		std::vector<LeafPattern*> leaves();

		// Attempt to find something in 'left' that matches this pattern's spec, and if so, move it to 'collected'
		virtual bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const = 0;

		virtual std::string const& name() const = 0;

		virtual bool hasValue() const { return false; }

		virtual size_t hash() const = 0;

		virtual ~Pattern() = default;
	};

	class LeafPattern
	: public Pattern {
	public:
		LeafPattern(std::string name, value v = {})
		: fName(std::move(name)),
		  fValue(std::move(v))
		{}

		virtual std::vector<Pattern*> flat(bool (*filter)(Pattern const*)) override {
			if (filter(this)) {
				return { this };
			}
			return {};
		}

		virtual void collect_leaves(std::vector<LeafPattern*>& lst) override final {
			lst.push_back(this);
		}

		virtual bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const override;

		virtual bool hasValue() const override { return static_cast<bool>(fValue); }

		value const& getValue() const { return fValue; }
		void setValue(value&& v) { fValue = std::move(v); }

		virtual std::string const& name() const override { return fName; }

		virtual size_t hash() const override {
			size_t seed = typeid(*this).hash_code();
			hash_combine(seed, fName);
			hash_combine(seed, fValue);
			return seed;
		}

	protected:
		virtual std::pair<size_t, std::shared_ptr<LeafPattern>> single_match(PatternList const&) const = 0;

	private:
		std::string fName;
		value fValue;
	};

	class BranchPattern
	: public Pattern {
	public:
		BranchPattern(PatternList children = {})
		: fChildren(std::move(children))
		{}

		Pattern& fix() {
			UniquePatternSet patterns;
			fix_identities(patterns);
			fix_repeating_arguments();
			return *this;
		}

		virtual std::string const& name() const override {
			throw std::runtime_error("Logic error: name() shouldnt be called on a BranchPattern");
		}

		virtual value const& getValue() const {
			throw std::runtime_error("Logic error: name() shouldnt be called on a BranchPattern");
		}

		virtual std::vector<Pattern*> flat(bool (*filter)(Pattern const*)) override {
			if (filter(this)) {
				return {this};
			}

			std::vector<Pattern*> ret;
			for(auto& child : fChildren) {
				auto sublist = child->flat(filter);
				ret.insert(ret.end(), sublist.begin(), sublist.end());
			}
			return ret;
		}

		virtual void collect_leaves(std::vector<LeafPattern*>& lst) override final {
			for(auto& child : fChildren) {
				child->collect_leaves(lst);
			}
		}

		void setChildren(PatternList children) {
			fChildren = std::move(children);
		}

		PatternList const& children() const { return fChildren; }

		virtual void fix_identities(UniquePatternSet& patterns) {
			for(auto& child : fChildren) {
				// this will fix up all its children, if needed
				if (auto bp = dynamic_cast<BranchPattern*>(child.get())) {
					bp->fix_identities(patterns);
				}

				// then we try to add it to the list
				auto inserted = patterns.insert(child);
				if (!inserted.second) {
					// already there? then reuse the existing shared_ptr for that thing
					child = *inserted.first;
				}
			}
		}

		virtual size_t hash() const override {
			size_t seed = typeid(*this).hash_code();
			hash_combine(seed, fChildren.size());
			for(auto const& child : fChildren) {
				hash_combine(seed, child->hash());
			}
			return seed;
		}
	private:
		void fix_repeating_arguments();

	protected:
		PatternList fChildren;
	};

	class Argument
	: public LeafPattern {
	public:
		using LeafPattern::LeafPattern;

	protected:
		virtual std::pair<size_t, std::shared_ptr<LeafPattern>> single_match(PatternList const& left) const override;
	};

	class Command : public Argument {
	public:
		Command(std::string name, value v = value{false})
		: Argument(std::move(name), std::move(v))
		{}

	protected:
		virtual std::pair<size_t, std::shared_ptr<LeafPattern>> single_match(PatternList const& left) const override;
	};

	class Option final
	: public LeafPattern
	{
	public:
		static Option parse(std::string const& option_description);

		Option(std::string shortOption,
		       std::string longOption,
		       int argcount = 0,
		       value v = value{false})
		: LeafPattern(longOption.empty() ? shortOption : longOption,
			      std::move(v)),
		  fShortOption(std::move(shortOption)),
		  fLongOption(std::move(longOption)),
		  fArgcount(argcount)
		{
			// From Python:
			//   self.value = None if value is False and argcount else value
			if (argcount && v.isBool() && !v.asBool()) {
				setValue(value{});
			}
		}

		Option(Option const&) = default;
		Option(Option&&) = default;
		Option& operator=(Option const&) = default;
		Option& operator=(Option&&) = default;

		using LeafPattern::setValue;

		std::string const& longOption() const { return fLongOption; }
		std::string const& shortOption() const { return fShortOption; }
		int argCount() const { return fArgcount; }

		virtual size_t hash() const override {
			size_t seed = LeafPattern::hash();
			hash_combine(seed, fShortOption);
			hash_combine(seed, fLongOption);
			hash_combine(seed, fArgcount);
			return seed;
		}

	protected:
		virtual std::pair<size_t, std::shared_ptr<LeafPattern>> single_match(PatternList const& left) const override;

	private:
		std::string fShortOption;
		std::string fLongOption;
		int fArgcount;
	};

	class Required : public BranchPattern {
	public:
		using BranchPattern::BranchPattern;

		bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const override;
	};

	class Optional : public BranchPattern {
	public:
		using BranchPattern::BranchPattern;

		bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const override {
			for(auto const& pattern : fChildren) {
				pattern->match(left, collected);
			}
			return true;
		}
	};

	class OptionsShortcut : public Optional {
		using Optional::Optional;
	};

	class OneOrMore : public BranchPattern {
	public:
		using BranchPattern::BranchPattern;

		bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const override;
	};

	class Either : public BranchPattern {
	public:
		using BranchPattern::BranchPattern;

		bool match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const override;
	};

	inline std::vector<LeafPattern*> Pattern::leaves()
	{
		std::vector<LeafPattern*> ret;
		collect_leaves(ret);
		return ret;
	}

	static inline std::vector<PatternList> transform(PatternList pattern)
	{
		std::vector<PatternList> result;

		std::vector<PatternList> groups;
		groups.emplace_back(std::move(pattern));

		while(!groups.empty()) {
			// pop off the first element
			auto children = std::move(groups[0]);
			groups.erase(groups.begin());

			// find the first branch node in the list
			auto child_iter = std::find_if(children.begin(), children.end(), [](std::shared_ptr<Pattern> const& p) {
				return dynamic_cast<BranchPattern const*>(p.get());
			});

			// no branch nodes left : expansion is complete for this grouping
			if (child_iter == children.end()) {
				result.emplace_back(std::move(children));
				continue;
			}

			// pop the child from the list
			auto child = std::move(*child_iter);
			children.erase(child_iter);

			// expand the branch in the appropriate way
			if (Either* either = dynamic_cast<Either*>(child.get())) {
				// "[e] + children" for each child 'e' in Either
				for(auto const& eitherChild : either->children()) {
					PatternList group = { eitherChild };
					group.insert(group.end(), children.begin(), children.end());

					groups.emplace_back(std::move(group));
				}
			} else if (OneOrMore* oneOrMore = dynamic_cast<OneOrMore*>(child.get())) {
				// child.children * 2 + children
				auto const& subchildren = oneOrMore->children();
				PatternList group = subchildren;
				group.insert(group.end(), subchildren.begin(), subchildren.end());
				group.insert(group.end(), children.begin(), children.end());

				groups.emplace_back(std::move(group));
			} else { // Required, Optional, OptionsShortcut
				BranchPattern* branch = dynamic_cast<BranchPattern*>(child.get());

				// child.children + children
				PatternList group = branch->children();
				group.insert(group.end(), children.begin(), children.end());

				groups.emplace_back(std::move(group));
			}
		}

		return result;
	}

	inline void BranchPattern::fix_repeating_arguments()
	{
		std::vector<PatternList> either = transform(children());
		for(auto const& group : either) {
			// use multiset to help identify duplicate entries
			std::unordered_multiset<std::shared_ptr<Pattern>, PatternHasher> group_set {group.begin(), group.end()};
			for(auto const& e : group_set) {
				if (group_set.count(e) == 1)
					continue;

				LeafPattern* leaf = dynamic_cast<LeafPattern*>(e.get());
				if (!leaf) continue;

				bool ensureList = false;
				bool ensureInt = false;

				if (dynamic_cast<Command*>(leaf)) {
					ensureInt = true;
				} else if (dynamic_cast<Argument*>(leaf)) {
					ensureList = true;
				} else if (Option* o = dynamic_cast<Option*>(leaf)) {
					if (o->argCount()) {
						ensureList = true;
					} else {
						ensureInt = true;
					}
				}

				if (ensureList) {
					std::vector<std::string> newValue;
					if (leaf->getValue().isString()) {
						newValue = split(leaf->getValue().asString());
					}
					if (!leaf->getValue().isStringList()) {
						leaf->setValue(value{newValue});
					}
				} else if (ensureInt) {
					leaf->setValue(value{0});
				}
			}
		}
	}

	inline bool LeafPattern::match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const
	{
		auto match = single_match(left);
		if (!match.second) {
			return false;
		}

		left.erase(left.begin()+static_cast<std::ptrdiff_t>(match.first));

		auto same_name = std::find_if(collected.begin(), collected.end(), [&](std::shared_ptr<LeafPattern> const& p) {
			return p->name()==name();
		});
		if (getValue().isLong()) {
			long val = 1;
			if (same_name == collected.end()) {
				collected.push_back(match.second);
				match.second->setValue(value{val});
			} else if ((**same_name).getValue().isLong()) {
				val += (**same_name).getValue().asLong();
				(**same_name).setValue(value{val});
			} else {
				(**same_name).setValue(value{val});
			}
		} else if (getValue().isStringList()) {
			std::vector<std::string> val;
			if (match.second->getValue().isString()) {
				val.push_back(match.second->getValue().asString());
			} else if (match.second->getValue().isStringList()) {
				val = match.second->getValue().asStringList();
			} else {
				/// cant be!?
			}

			if (same_name == collected.end()) {
				collected.push_back(match.second);
				match.second->setValue(value{val});
			} else if ((**same_name).getValue().isStringList()) {
				std::vector<std::string> const& list = (**same_name).getValue().asStringList();
				val.insert(val.begin(), list.begin(), list.end());
				(**same_name).setValue(value{val});
			} else {
				(**same_name).setValue(value{val});
			}
		} else {
			collected.push_back(match.second);
		}
		return true;
	}

	inline std::pair<size_t, std::shared_ptr<LeafPattern>> Argument::single_match(PatternList const& left) const
	{
		std::pair<size_t, std::shared_ptr<LeafPattern>> ret {};

		for(size_t i = 0, size = left.size(); i < size; ++i)
		{
			auto arg = dynamic_cast<Argument const*>(left[i].get());
			if (arg) {
				ret.first = i;
				ret.second = std::make_shared<Argument>(name(), arg->getValue());
				break;
			}
		}

		return ret;
	}

	inline std::pair<size_t, std::shared_ptr<LeafPattern>> Command::single_match(PatternList const& left) const
	{
		std::pair<size_t, std::shared_ptr<LeafPattern>> ret {};

		for(size_t i = 0, size = left.size(); i < size; ++i)
		{
			auto arg = dynamic_cast<Argument const*>(left[i].get());
			if (arg) {
				if (name() == arg->getValue()) {
					ret.first = i;
					ret.second = std::make_shared<Command>(name(), value{true});
				}
				break;
			}
		}

		return ret;
	}

	inline Option Option::parse(std::string const& option_description)
	{
		std::string shortOption, longOption;
		int argcount = 0;
		value val { false };

		auto double_space = option_description.find("  ");
		auto options_end = option_description.end();
		if (double_space != std::string::npos) {
			options_end = option_description.begin() + static_cast<std::ptrdiff_t>(double_space);
		}

		static const std::regex pattern {"(-{1,2})?(.*?)([,= ]|$)"};
		for(std::sregex_iterator i {option_description.begin(), options_end, pattern, std::regex_constants::match_not_null},
			   e{};
			i != e;
			++i)
		{
			std::smatch const& match = *i;
			if (match[1].matched) { // [1] is optional.
				if (match[1].length()==1) {
						shortOption = "-" + match[2].str();
				} else {
						longOption =  "--" + match[2].str();
				}
			} else if (match[2].length() > 0) { // [2] always matches.
				std::string m = match[2];
				argcount = 1;
			} else {
				// delimeter
			}

			if (match[3].length() == 0) { // [3] always matches.
				// Hit end of string. For some reason 'match_not_null' will let us match empty
				// at the end, and then we'll spin in an infinite loop. So, if we hit an empty
				// match, we know we must be at the end.
				break;
			}
		}

		if (argcount) {
			std::smatch match;
			if (std::regex_search(options_end, option_description.end(),
						  match,
						  std::regex{"\\[default: (.*)\\]", std::regex::icase}))
			{
				val = match[1].str();
			}
		}

		return {std::move(shortOption),
			std::move(longOption),
			argcount,
			std::move(val)};
	}

	inline std::pair<size_t, std::shared_ptr<LeafPattern>> Option::single_match(PatternList const& left) const
	{
		auto thematch = find_if(left.begin(), left.end(), [this](std::shared_ptr<Pattern> const& a) {
			auto leaf = std::dynamic_pointer_cast<LeafPattern>(a);
			return leaf && this->name() == leaf->name();
		});
		if (thematch == left.end()) {
			return {};
		}
		return { std::distance(left.begin(), thematch), std::dynamic_pointer_cast<LeafPattern>(*thematch) };
	}

	inline bool Required::match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const {
		auto l = left;
		auto c = collected;
		for(auto const& pattern : fChildren) {
			bool ret = pattern->match(l, c);
			if (!ret) {
				// leave (left, collected) untouched
				return false;
			}
		}

		left = std::move(l);
		collected = std::move(c);
		return true;
	}

	inline bool OneOrMore::match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const
	{
		assert(fChildren.size() == 1);

		auto l = left;
		auto c = collected;

		bool matched = true;
		size_t times = 0;

		decltype(l) l_;
		bool firstLoop = true;

		while (matched) {
			// could it be that something didn't match but changed l or c?
			matched = fChildren[0]->match(l, c);

			if (matched)
				++times;

			if (firstLoop) {
				firstLoop = false;
			} else if (l == l_) {
				break;
			}

			l_ = l;
		}

		if (times == 0) {
			return false;
		}

		left = std::move(l);
		collected = std::move(c);
		return true;
	}

	inline bool Either::match(PatternList& left, std::vector<std::shared_ptr<LeafPattern>>& collected) const
	{
		using Outcome = std::pair<PatternList, std::vector<std::shared_ptr<LeafPattern>>>;

		std::vector<Outcome> outcomes;

		for(auto const& pattern : fChildren) {
			// need a copy so we apply the same one for every iteration
			auto l = left;
			auto c = collected;
			bool matched = pattern->match(l, c);
			if (matched) {
				outcomes.emplace_back(std::move(l), std::move(c));
			}
		}

		auto min = std::min_element(outcomes.begin(), outcomes.end(), [](Outcome const& o1, Outcome const& o2) {
			return o1.first.size() < o2.first.size();
		});

		if (min == outcomes.end()) {
			// (left, collected) unchanged
			return false;
		}

		std::tie(left, collected) = std::move(*min);
		return true;
	}

}


using namespace docopt;

DOCOPT_INLINE
std::ostream& docopt::operator<<(std::ostream& os, value const& val)
{
    if (val.isBool()) {
        bool b = val.asBool();
        os << (b ? "true" : "false");
    } else if (val.isLong()) {
        long v = val.asLong();
        os << v;
    } else if (val.isString()) {
        std::string const& str = val.asString();
        os << '"' << str << '"';
    } else if (val.isStringList()) {
        auto const& list = val.asStringList();
        os << "[";
        bool first = true;
        for(auto const& el : list) {
            if (first) {
                first = false;
            } else {
                os << ", ";
            }
            os << '"' << el << '"';
        }
        os << "]";
    } else {
        os << "null";
    }
    return os;
}

class Tokens {
public:
    Tokens(std::vector<std::string> tokens, bool isParsingArgv = true)
    : fTokens(std::move(tokens)),
      fIsParsingArgv(isParsingArgv)
    {}

    explicit operator bool() const {
        return fIndex < fTokens.size();
    }

    static Tokens from_pattern(std::string const& source) {
        static const std::regex re_separators {
            "(?:\\s*)" // any spaces (non-matching subgroup)
            "("
            "[\\[\\]\\(\\)\\|]" // one character of brackets or parens or pipe character
            "|"
            "\\.\\.\\."  // elipsis
            ")" };

        static const std::regex re_strings {
            "(?:\\s*)" // any spaces (non-matching subgroup)
            "("
            "\\S*<.*?>"  // strings, but make sure to keep "< >" strings together
            "|"
            "[^<>\\s]+"     // string without <>
            ")" };

        // We do two stages of regex matching. The '[]()' and '...' are strong delimeters
        // and need to be split out anywhere they occur (even at the end of a token). We
        // first split on those, and then parse the stuff between them to find the string
        // tokens. This is a little harder than the python version, since they have regex.split
        // and we dont have anything like that.

        std::vector<std::string> tokens;
        std::for_each(std::sregex_iterator{ source.begin(), source.end(), re_separators },
                  std::sregex_iterator{},
                  [&](std::smatch const& match)
                  {
                      // handle anything before the separator (this is the "stuff" between the delimeters)
                      if (match.prefix().matched) {
                          std::for_each(std::sregex_iterator{match.prefix().first, match.prefix().second, re_strings},
                                std::sregex_iterator{},
                                [&](std::smatch const& m)
                                {
                                    tokens.push_back(m[1].str());
                                });
                      }

                      // handle the delimter token itself
                      if (match[1].matched) {
                          tokens.push_back(match[1].str());
                      }
                  });

        return Tokens(tokens, false);
    }

    std::string const& current() const {
        if (*this)
            return fTokens[fIndex];

        static std::string const empty;
        return empty;
    }

    std::string the_rest() const {
        if (!*this)
            return {};
        return join(fTokens.begin()+static_cast<std::ptrdiff_t>(fIndex),
                fTokens.end(),
                " ");
    }

    std::string pop() {
        return std::move(fTokens.at(fIndex++));
    }

    bool isParsingArgv() const { return fIsParsingArgv; }

    struct OptionError : std::runtime_error { using runtime_error::runtime_error; };

private:
    std::vector<std::string> fTokens;
    size_t fIndex = 0;
    bool fIsParsingArgv;
};

// Get all instances of 'T' from the pattern
template <typename T>
std::vector<T*> flat_filter(Pattern& pattern) {
    std::vector<Pattern*> flattened = pattern.flat([](Pattern const* p) -> bool {
        return dynamic_cast<T const*>(p) != nullptr;
    });

    // now, we're guaranteed to have T*'s, so just use static_cast
    std::vector<T*> ret;
    std::transform(flattened.begin(), flattened.end(), std::back_inserter(ret), [](Pattern* p) {
        return static_cast<T*>(p);
    });
    return ret;
}

static std::vector<std::string> parse_section(std::string const& name, std::string const& source) {
    // ECMAScript regex only has "?=" for a non-matching lookahead. In order to make sure we always have
    // a newline to anchor our matching, we have to avoid matching the final newline of each grouping.
    // Therefore, our regex is adjusted from the docopt Python one to use ?= to match the newlines before
    // the following lines, rather than after.
    std::regex const re_section_pattern {
        "(?:^|\\n)"  // anchored at a linebreak (or start of string)
        "("
           "[^\\n]*" + name + "[^\\n]*(?=\\n?)" // a line that contains the name
           "(?:\\n[ \\t].*?(?=\\n|$))*"         // followed by any number of lines that are indented
        ")",
        std::regex::icase
    };

    std::vector<std::string> ret;
    std::for_each(std::sregex_iterator(source.begin(), source.end(), re_section_pattern),
              std::sregex_iterator(),
              [&](std::smatch const& match)
    {
        ret.push_back(trim(match[1].str()));
    });

    return ret;
}

static bool is_argument_spec(std::string const& token) {
    if (token.empty())
        return false;

    if (token[0]=='<' && token[token.size()-1]=='>')
        return true;

    if (std::all_of(token.begin(), token.end(), &::isupper))
        return true;

    return false;
}

template <typename I>
std::vector<std::string> longOptions(I iter, I end) {
    std::vector<std::string> ret;
    std::transform(iter, end,
               std::back_inserter(ret),
               [](typename I::reference opt) { return opt->longOption(); });
    return ret;
}

static PatternList parse_long(Tokens& tokens, std::vector<Option>& options)
{
    // long ::= '--' chars [ ( ' ' | '=' ) chars ] ;
    std::string longOpt, equal;
    value val;
    std::tie(longOpt, equal, val) = partition(tokens.pop(), "=");

    assert(starts_with(longOpt, "--"));

    if (equal.empty()) {
        val = value{};
    }

    // detect with options match this long option
    std::vector<Option const*> similar;
    for(auto const& option : options) {
        if (option.longOption()==longOpt)
            similar.push_back(&option);
    }

    // maybe allow similar options that match by prefix
    if (tokens.isParsingArgv() && similar.empty()) {
        for(auto const& option : options) {
            if (option.longOption().empty())
                continue;
            if (starts_with(option.longOption(), longOpt))
                similar.push_back(&option);
        }
    }

    PatternList ret;

    if (similar.size() > 1) { // might be simply specified ambiguously 2+ times?
        std::vector<std::string> prefixes = longOptions(similar.begin(), similar.end());
        std::string error = "'" + longOpt + "' is not a unique prefix: ";
        error.append(join(prefixes.begin(), prefixes.end(), ", "));
        throw Tokens::OptionError(std::move(error));
    } else if (similar.empty()) {
        int argcount = equal.empty() ? 0 : 1;
        options.emplace_back("", longOpt, argcount);

        auto o = std::make_shared<Option>(options.back());
        if (tokens.isParsingArgv()) {
            o->setValue(argcount ? value{val} : value{true});
        }
        ret.push_back(o);
    } else {
        auto o = std::make_shared<Option>(*similar[0]);
        if (o->argCount() == 0) {
            if (val) {
                std::string error = o->longOption() + " must not have an argument";
                throw Tokens::OptionError(std::move(error));
            }
        } else {
            if (!val) {
                auto const& token = tokens.current();
                if (token.empty() || token=="--") {
                    std::string error = o->longOption() + " requires an argument";
                    throw Tokens::OptionError(std::move(error));
                }
                val = tokens.pop();
            }
        }
        if (tokens.isParsingArgv()) {
            o->setValue(val ? std::move(val) : value{true});
        }
        ret.push_back(o);
    }

    return ret;
}

static PatternList parse_short(Tokens& tokens, std::vector<Option>& options)
{
    // shorts ::= '-' ( chars )* [ [ ' ' ] chars ] ;

    auto token = tokens.pop();

    assert(starts_with(token, "-"));
    assert(!starts_with(token, "--"));

    auto i = token.begin();
    ++i; // skip the leading '-'

    PatternList ret;
    while (i != token.end()) {
        std::string shortOpt = { '-', *i };
        ++i;

        std::vector<Option const*> similar;
        for(auto const& option : options) {
            if (option.shortOption()==shortOpt)
                similar.push_back(&option);
        }

        if (similar.size() > 1) {
            std::string error = shortOpt + " is specified ambiguously "
            + std::to_string(similar.size()) + " times";
            throw Tokens::OptionError(std::move(error));
        } else if (similar.empty()) {
            options.emplace_back(shortOpt, "", 0);

            auto o = std::make_shared<Option>(options.back());
            if (tokens.isParsingArgv()) {
                o->setValue(value{true});
            }
            ret.push_back(o);
        } else {
            auto o = std::make_shared<Option>(*similar[0]);
            value val;
            if (o->argCount()) {
                if (i == token.end()) {
                    // consume the next token
                    auto const& ttoken = tokens.current();
                    if (ttoken.empty() || ttoken=="--") {
                        std::string error = shortOpt + " requires an argument";
                        throw Tokens::OptionError(std::move(error));
                    }
                    val = tokens.pop();
                } else {
                    // consume all the rest
                    val = std::string{i, token.end()};
                    i = token.end();
                }
            }

            if (tokens.isParsingArgv()) {
                o->setValue(val ? std::move(val) : value{true});
            }
            ret.push_back(o);
        }
    }

    return ret;
}

static PatternList parse_expr(Tokens& tokens, std::vector<Option>& options);

static PatternList parse_atom(Tokens& tokens, std::vector<Option>& options)
{
    // atom ::= '(' expr ')' | '[' expr ']' | 'options'
    //             | long | shorts | argument | command ;

    std::string const& token = tokens.current();

    PatternList ret;

    if (token == "[") {
        tokens.pop();

        auto expr = parse_expr(tokens, options);

        auto trailing = tokens.pop();
        if (trailing != "]") {
            throw DocoptLanguageError("Mismatched '['");
        }

        ret.emplace_back(std::make_shared<Optional>(std::move(expr)));
    } else if (token=="(") {
        tokens.pop();

        auto expr = parse_expr(tokens, options);

        auto trailing = tokens.pop();
        if (trailing != ")") {
            throw DocoptLanguageError("Mismatched '('");
        }

        ret.emplace_back(std::make_shared<Required>(std::move(expr)));
    } else if (token == "options") {
        tokens.pop();
        ret.emplace_back(std::make_shared<OptionsShortcut>());
    } else if (starts_with(token, "--") && token != "--") {
        ret = parse_long(tokens, options);
    } else if (starts_with(token, "-") && token != "-" && token != "--") {
        ret = parse_short(tokens, options);
    } else if (is_argument_spec(token)) {
        ret.emplace_back(std::make_shared<Argument>(tokens.pop()));
    } else {
        ret.emplace_back(std::make_shared<Command>(tokens.pop()));
    }

    return ret;
}

static PatternList parse_seq(Tokens& tokens, std::vector<Option>& options)
{
    // seq ::= ( atom [ '...' ] )* ;"""

    PatternList ret;

    while (tokens) {
        auto const& token = tokens.current();

        if (token=="]" || token==")" || token=="|")
            break;

        auto atom = parse_atom(tokens, options);
        if (tokens.current() == "...") {
            ret.emplace_back(std::make_shared<OneOrMore>(std::move(atom)));
            tokens.pop();
        } else {
            std::move(atom.begin(), atom.end(), std::back_inserter(ret));
        }
    }

    return ret;
}

static std::shared_ptr<Pattern> maybe_collapse_to_required(PatternList&& seq)
{
    if (seq.size()==1) {
        return std::move(seq[0]);
    }
    return std::make_shared<Required>(std::move(seq));
}

static std::shared_ptr<Pattern> maybe_collapse_to_either(PatternList&& seq)
{
    if (seq.size()==1) {
        return std::move(seq[0]);
    }
    return std::make_shared<Either>(std::move(seq));
}

PatternList parse_expr(Tokens& tokens, std::vector<Option>& options)
{
    // expr ::= seq ( '|' seq )* ;

    auto seq = parse_seq(tokens, options);

    if (tokens.current() != "|")
        return seq;

    PatternList ret;
    ret.emplace_back(maybe_collapse_to_required(std::move(seq)));

    while (tokens.current() == "|") {
        tokens.pop();
        seq = parse_seq(tokens, options);
        ret.emplace_back(maybe_collapse_to_required(std::move(seq)));
    }

    return { maybe_collapse_to_either(std::move(ret)) };
}

static Required parse_pattern(std::string const& source, std::vector<Option>& options)
{
    auto tokens = Tokens::from_pattern(source);
    auto result = parse_expr(tokens, options);

    if (tokens)
        throw DocoptLanguageError("Unexpected ending: '" + tokens.the_rest() + "'");

    assert(result.size() == 1  &&  "top level is always one big");
    return Required{ std::move(result) };
}


static std::string formal_usage(std::string const& section) {
    std::string ret = "(";

    auto i = section.find(':')+1;  // skip past "usage:"
    auto parts = split(section, i);
    for(size_t ii = 1; ii < parts.size(); ++ii) {
        if (parts[ii] == parts[0]) {
            ret += " ) | (";
        } else {
            ret.push_back(' ');
            ret += parts[ii];
        }
    }

    ret += " )";
    return ret;
}

static PatternList parse_argv(Tokens tokens, std::vector<Option>& options, bool options_first)
{
    // Parse command-line argument vector.
    //
    // If options_first:
    //    argv ::= [ long | shorts ]* [ argument ]* [ '--' [ argument ]* ] ;
    // else:
    //    argv ::= [ long | shorts | argument ]* [ '--' [ argument ]* ] ;

    PatternList ret;
    while (tokens) {
        auto const& token = tokens.current();

        if (token=="--") {
            // option list is done; convert all the rest to arguments
            while (tokens) {
                ret.emplace_back(std::make_shared<Argument>("", tokens.pop()));
            }
        } else if (starts_with(token, "--")) {
            auto&& parsed = parse_long(tokens, options);
            std::move(parsed.begin(), parsed.end(), std::back_inserter(ret));
        } else if (token[0]=='-' && token != "-") {
            auto&& parsed = parse_short(tokens, options);
            std::move(parsed.begin(), parsed.end(), std::back_inserter(ret));
        } else if (options_first) {
            // option list is done; convert all the rest to arguments
            while (tokens) {
                ret.emplace_back(std::make_shared<Argument>("", tokens.pop()));
            }
        } else {
            ret.emplace_back(std::make_shared<Argument>("", tokens.pop()));
        }
    }

    return ret;
}

std::vector<Option> parse_defaults(std::string const& doc) {
    // This pattern is a delimiter by which we split the options.
    // The delimiter is a new line followed by a whitespace(s) followed by one or two hyphens.
    static std::regex const re_delimiter{
        "(?:^|\\n)[ \\t]*"  // a new line with leading whitespace
        "(?=-{1,2})"        // [split happens here] (positive lookahead) ... and followed by one or two hyphes
    };

    std::vector<Option> defaults;
    for (auto s : parse_section("options:", doc)) {
        s.erase(s.begin(), s.begin() + static_cast<std::ptrdiff_t>(s.find(':')) + 1); // get rid of "options:"

        for (const auto& opt : regex_split(s, re_delimiter)) {
            if (starts_with(opt, "-")) {
                defaults.emplace_back(Option::parse(opt));
            }
        }
    }

    return defaults;
}

static bool isOptionSet(PatternList const& options, std::string const& opt1, std::string const& opt2 = "") {
    return std::any_of(options.begin(), options.end(), [&](std::shared_ptr<Pattern const> const& opt) -> bool {
        auto const& name = opt->name();
        if (name==opt1 || (!opt2.empty() && name==opt2)) {
            return opt->hasValue();
        }
        return false;
    });
}

static void extras(bool help, bool version, PatternList const& options) {
    if (help && isOptionSet(options, "-h", "--help")) {
        throw DocoptExitHelp();
    }

    if (version && isOptionSet(options, "--version")) {
        throw DocoptExitVersion();
    }
}

// Parse the doc string and generate the Pattern tree
static std::pair<Required, std::vector<Option>> create_pattern_tree(std::string const& doc)
{
    auto usage_sections = parse_section("usage:", doc);
    if (usage_sections.empty()) {
        throw DocoptLanguageError("'usage:' (case-insensitive) not found.");
    }
    if (usage_sections.size() > 1) {
        throw DocoptLanguageError("More than one 'usage:' (case-insensitive).");
    }

    std::vector<Option> options = parse_defaults(doc);
    Required pattern = parse_pattern(formal_usage(usage_sections[0]), options);

    std::vector<Option const*> pattern_options = flat_filter<Option const>(pattern);

    using UniqueOptions = std::unordered_set<Option const*, PatternHasher, PatternPointerEquality>;
    UniqueOptions const uniq_pattern_options { pattern_options.begin(), pattern_options.end() };

    // Fix up any "[options]" shortcuts with the actual option tree
    for(auto& options_shortcut : flat_filter<OptionsShortcut>(pattern)) {
        std::vector<Option> doc_options = parse_defaults(doc);

        // set(doc_options) - set(pattern_options)
        UniqueOptions uniq_doc_options;
        for(auto const& opt : doc_options) {
            if (uniq_pattern_options.count(&opt))
                continue;
            uniq_doc_options.insert(&opt);
        }

        // turn into shared_ptr's and set as children
        PatternList children;
        std::transform(uniq_doc_options.begin(), uniq_doc_options.end(),
                   std::back_inserter(children), [](Option const* opt) {
                       return std::make_shared<Option>(*opt);
                   });
        options_shortcut->setChildren(std::move(children));
    }

    return { std::move(pattern), std::move(options) };
}

DOCOPT_INLINE
std::map<std::string, value>
docopt::docopt_parse(std::string const& doc,
             std::vector<std::string> const& argv,
             bool help,
             bool version,
             bool options_first)
{
    Required pattern;
    std::vector<Option> options;
    try {
        std::tie(pattern, options) = create_pattern_tree(doc);
    } catch (Tokens::OptionError const& error) {
        throw DocoptLanguageError(error.what());
    }

    PatternList argv_patterns;
    try {
        argv_patterns = parse_argv(Tokens(argv), options, options_first);
    } catch (Tokens::OptionError const& error) {
        throw DocoptArgumentError(error.what());
    }

    extras(help, version, argv_patterns);

    std::vector<std::shared_ptr<LeafPattern>> collected;
    bool matched = pattern.fix().match(argv_patterns, collected);
    if (matched && argv_patterns.empty()) {
        std::map<std::string, value> ret;

        // (a.name, a.value) for a in (pattern.flat() + collected)
        for (auto* p : pattern.leaves()) {
            ret[p->name()] = p->getValue();
        }

        for (auto const& p : collected) {
            ret[p->name()] = p->getValue();
        }

        return ret;
    }

    if (matched) {
        std::string leftover = join(argv.begin(), argv.end(), ", ");
        throw DocoptArgumentError("Unexpected argument: " + leftover);
    }

    throw DocoptArgumentError("Arguments did not match expected patterns"); // BLEH. Bad error.
}

DOCOPT_INLINE
std::map<std::string, value>
docopt::docopt(std::string const& doc,
           std::vector<std::string> const& argv,
           bool help,
           std::string const& version,
           bool options_first) noexcept
{
    try {
        return docopt_parse(doc, argv, help, !version.empty(), options_first);
    } catch (DocoptExitHelp const&) {
        std::cout << doc << std::endl;
        std::exit(0);
    } catch (DocoptExitVersion const&) {
        std::cout << version << std::endl;
        std::exit(0);
    } catch (DocoptLanguageError const& error) {
        std::cerr << "Docopt usage string could not be parsed" << std::endl;
        std::cerr << error.what() << std::endl;
        std::exit(-1);
    } catch (DocoptArgumentError const& error) {
        std::cerr << error.what();
        std::cout << std::endl;
        std::cout << doc << std::endl;
        std::exit(-1);
    } /* Any other exception is unexpected: let std::terminate grab it */
}

#endif /* defined(docopt__docopt_h_) */

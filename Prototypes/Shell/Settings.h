// TrackUnlimited Phase 3.5: settings and input bindings.
// Plain C++17, no engine dependency.
//
// Small, and two rules in it are the difference between a config system that ages
// well and one that loses somebody's preferences on an update.
//
// ===================== A DEFAULT IS NOT A VALUE =====================
//
// The obvious implementation writes every setting to the file. Then the default
// can never change: everybody who ever launched the application has the old one
// written down as though they chose it, and shipping a better default reaches
// only new users.
//
// So ONLY WHAT WAS EXPLICITLY SET IS WRITTEN. Everything else resolves to the
// current default at read time, which means improving a default improves it for
// everybody who never touched it — and leaves it alone for everybody who did.
//
// ===================== UNKNOWN KEYS SURVIVE =====================
//
// A config file written by a newer version contains keys this one has never heard
// of. Dropping them on load and rewriting the file DESTROYS THEM — so running an
// older build once, for any reason, silently resets everything the newer one
// added.
//
// They are kept and written back untouched. It costs a map and it is the
// difference between "I ran the old version to check something" being harmless
// and being an afternoon.

#pragma once

#include <algorithm>
#include <map>
#include <string>
#include <vector>

class FSettings
{
public:
    // A default is DECLARED, not written. Declaring one twice is the caller's
    // business; the last wins, as it would in code.
    void Declare(const std::string& Key, const std::string& DefaultValue)
    {
        Default[Key] = DefaultValue;
    }

    // What a caller actually reads. Explicit value if there is one, otherwise the
    // CURRENT default — so improving a default improves it for everybody who never
    // touched that setting.
    std::string Get(const std::string& Key) const
    {
        const auto E = Explicit.find(Key);
        if (E != Explicit.end()) { return E->second; }
        const auto D = Default.find(Key);
        return D != Default.end() ? D->second : std::string();
    }

    bool GetBool(const std::string& Key) const
    {
        const std::string V = Get(Key);
        return V == "true" || V == "1";
    }
    double GetNumber(const std::string& Key) const
    {
        const std::string V = Get(Key);
        return V.empty() ? 0.0 : std::atof(V.c_str());
    }

    void Set(const std::string& Key, const std::string& Value) { Explicit[Key] = Value; }

    // Back to the default, which is a DELETION rather than writing the default's
    // current text. Writing it would freeze today's default into the file and
    // undo the whole point.
    void Reset(const std::string& Key) { Explicit.erase(Key); }
    void ResetAll() { Explicit.clear(); }

    bool WasSetExplicitly(const std::string& Key) const
    {
        return Explicit.find(Key) != Explicit.end();
    }
    bool IsAtDefault(const std::string& Key) const { return !WasSetExplicitly(Key); }
    std::size_t NumExplicit() const { return Explicit.size(); }

    // ===================== LOAD AND SAVE =====================
    //
    // A trivial `key=value` line format. Not JSON, because a settings file is one
    // of the few things a person edits by hand when something has gone wrong, and
    // a stray comma should not cost them the file.

    void Load(const std::string& Text)
    {
        Explicit.clear();
        Unknown.clear();
        std::size_t Pos = 0;
        while (Pos < Text.size())
        {
            std::size_t End = Text.find('\n', Pos);
            if (End == std::string::npos) { End = Text.size(); }
            const std::string Line = Trim(Text.substr(Pos, End - Pos));
            Pos = End + 1;
            if (Line.empty() || Line[0] == '#') { continue; }

            const std::size_t Eq = Line.find('=');
            if (Eq == std::string::npos) { continue; }
            const std::string Key = Trim(Line.substr(0, Eq));
            const std::string Value = Trim(Line.substr(Eq + 1));
            if (Key.empty()) { continue; }

            // UNKNOWN KEYS SURVIVE. A file written by a newer version has settings
            // this build has never heard of, and dropping them on load then
            // rewriting DESTROYS THEM — so running the older build once silently
            // resets everything the newer one added.
            if (Default.find(Key) == Default.end())
            {
                Unknown[Key] = Value;
            }
            else
            {
                Explicit[Key] = Value;
            }
        }
    }

    std::string Save() const
    {
        std::string Out = "# TrackUnlimited settings. Only what you changed is here;\n"
                          "# everything else follows the application's current default.\n";
        for (const auto& KV : Explicit) { Out += KV.first + " = " + KV.second + "\n"; }
        if (!Unknown.empty())
        {
            Out += "\n# Written by a newer version. Kept untouched so running this\n"
                   "# build does not discard them.\n";
            for (const auto& KV : Unknown) { Out += KV.first + " = " + KV.second + "\n"; }
        }
        return Out;
    }

    std::size_t NumUnknownKept() const { return Unknown.size(); }

private:
    static std::string Trim(const std::string& In)
    {
        std::size_t A = 0, B = In.size();
        while (A < B && (In[A] == ' ' || In[A] == '\t' || In[A] == '\r')) { ++A; }
        while (B > A && (In[B - 1] == ' ' || In[B - 1] == '\t' || In[B - 1] == '\r')) { --B; }
        return In.substr(A, B - A);
    }

    std::map<std::string, std::string> Default;
    std::map<std::string, std::string> Explicit;
    std::map<std::string, std::string> Unknown;
};

// ===================== INPUT BINDINGS =====================
//
// An action, a key, and a CONTEXT. The context is the part people leave out, and
// it is why `[Space]` can be dispatch on the operator panel and something else
// entirely while riding without either being a mistake.
struct FBinding
{
    std::string Action;
    std::string Key;
    std::string Context;    // "" is global and conflicts with everything
};

// Two bindings that fight. Reported, never resolved — the same rule as everywhere
// else here, because which one somebody meant is not knowable.
struct FBindingConflict
{
    std::string Key;
    std::string Context;
    std::string ActionA;
    std::string ActionB;
};

class FInputMap
{
public:
    void Bind(const std::string& Action, const std::string& Key, const std::string& Context = "")
    {
        // REBINDING AN ACTION REPLACES ITS KEY rather than adding a second, because
        // an action bound twice is almost never what somebody meant by dragging a
        // key onto it — and if it is, they can bind an alias action.
        for (FBinding& B : Binding)
        {
            if (B.Action == Action && B.Context == Context) { B.Key = Key; return; }
        }
        Binding.push_back({Action, Key, Context});
    }

    void Unbind(const std::string& Action, const std::string& Context = "")
    {
        Binding.erase(std::remove_if(Binding.begin(), Binding.end(),
            [&](const FBinding& B) { return B.Action == Action && B.Context == Context; }),
            Binding.end());
    }

    std::string KeyFor(const std::string& Action, const std::string& Context = "") const
    {
        for (const FBinding& B : Binding)
        {
            if (B.Action == Action && B.Context == Context) { return B.Key; }
        }
        return std::string();
    }

    std::size_t Num() const { return Binding.size(); }
    const FBinding& At(std::size_t i) const { return Binding[i]; }

    // Every pair that fights. A GLOBAL binding conflicts with everything on that
    // key, in any context — which is the case people miss, because it looks fine
    // in whichever context they happen to be looking at.
    std::vector<FBindingConflict> Conflicts() const
    {
        std::vector<FBindingConflict> Out;
        for (std::size_t i = 0; i < Binding.size(); ++i)
        {
            for (std::size_t j = i + 1; j < Binding.size(); ++j)
            {
                const FBinding& A = Binding[i];
                const FBinding& B = Binding[j];
                if (A.Key != B.Key) { continue; }
                const bool bSameContext = A.Context == B.Context;
                const bool bEitherGlobal = A.Context.empty() || B.Context.empty();
                if (bSameContext || bEitherGlobal)
                {
                    Out.push_back({A.Key, A.Context.empty() ? B.Context : A.Context,
                                   A.Action, B.Action});
                }
            }
        }
        return Out;
    }

    bool HasConflicts() const { return !Conflicts().empty(); }

    // A BINDING THAT CONFLICTS IS STILL STORED. Refusing it would leave somebody
    // stuck part-way through remapping — they press a key, it is rejected, and the
    // action they wanted to move it off still holds it. Real remapping UIs let the
    // clash exist and show it, which is also the report-never-repair rule.
    //
    // ponytail: no modifier keys and no chords. Every binding this project has is
    // a single key, and a chord model is a real design decision rather than a
    // field to add speculatively.

private:
    std::vector<FBinding> Binding;
};

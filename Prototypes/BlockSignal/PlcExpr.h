// TrackUnlimited Tier 2: the override expression.
// Plain C++17, no dependencies.
//
// `CLAUDE.md` constraint 7: an override is a PURE EXPRESSION over the named
// process image. No statements, no assignment, no loops, no state across scans.
// Evaluated against the snapshot the scan already takes.
//
// ===================== WHY AN EXPRESSION AND NOT A LANGUAGE =====================
//
// Everything a scripting layer normally has to buy, this gets for nothing:
//
//   DETERMINISM     A pure function of a snapshot. It drops into FSimDigest
//                   unchanged, because there is nothing for it to remember.
//   AN EXECUTION    Cost is the size of the tree, known at parse time. There is
//   BUDGET          no loop to run away and no recursion to bound.
//   A SANDBOX       No heap, no closures, no references outside the image. There
//                   is nothing to escape into.
//
// What it CANNOT express is state and sequencing — "run forward 4 s, then
// reverse, then park backwards" needs variables that survive a scan. Nothing
// built or planned needs that, and the day something does is the trigger for
// revisiting Structured Text. The ST cards stay as the design of record.
//
// ===================== STRICT ST SPELLING FROM DAY ONE =====================
//
// AND / OR / XOR / NOT, `=` for equality and `<>` for inequality, MOD, and
// SEL(G, IN0, IN1) rather than a ternary — note the order, IN0 when G is FALSE,
// which is IEC 61131-3's and is the opposite way round from how most people read
// it. Identifiers are case-insensitive.
//
// A SUBSET WITH THE WRONG OPERATORS IS NOT A SUBSET. If this taught `!=` and
// `? :` then a later ST would be a replacement rather than an extension, and
// everything somebody learned here would be worth nothing in industry — which is
// the entire argument for the verbatim-standards choices in `CLAUDE.md` §
// "The control layer is a LAYER OF CHOICE".
//
// ===================== THE SAFETY RULE IS STRUCTURAL =====================
//
// An override may only make a permissive MORE RESTRICTIVE. That is not enforced
// by review or by validation: `Restrict()` ANDs the result onto the existing
// chain, so there is no syntax that loosens one. A downloaded track's override is
// safe by construction, not by inspection.

#pragma once

#include "PlcImage.h"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

// ------------------------------------------------------------------ tokens

enum class ETok
{
    End, Number, Name,
    LParen, RParen, Comma,
    Plus, Minus, Star, Slash, Mod,
    Lt, Le, Gt, Ge, Eq, Ne,
    And, Or, Xor, Not,
    True, False,
};

struct FTok
{
    ETok Kind = ETok::End;
    double Number = 0.0;
    std::string Text;
    std::size_t At = 0;      // byte offset, so an error can point at it
};

// ------------------------------------------------------------------ the tree
//
// A tagged node in a flat vector rather than a pointer graph: an expression is
// built once and evaluated a few hundred times a second, and a contiguous
// postorder walk is both faster and impossible to leak.

enum class EOp
{
    Const, Load,
    Neg, Not,
    Add, Sub, Mul, Div, Mod,
    Lt, Le, Gt, Ge, Eq, Ne,
    And, Or, Xor,
    Sel,
};

struct FNode
{
    EOp Op = EOp::Const;
    FPlcValue Value;             // Const
    std::size_t Slot = 0;        // Load
    int A = -1, B = -1, C = -1;  // child indices
    EPlcType Type = EPlcType::Bool;
};

// A parsed and BOUND expression. Bound means every name in it resolved to a slot
// in a specific image shape — so an override written for another layout fails
// here rather than reading a slot that now means something else.
class FPlcExpr
{
public:
    bool IsValid() const { return Root >= 0; }
    EPlcType ResultType() const { return Root >= 0 ? Node[static_cast<std::size_t>(Root)].Type
                                                   : EPlcType::Bool; }
    const std::string& Error() const { return Err; }
    std::size_t ErrorAt() const { return ErrPos; }
    std::size_t NodeCount() const { return Node.size(); }

    FPlcValue Evaluate(const FProcessImage& Image) const
    {
        if (Root < 0) { return FPlcValue::Boolean(false); }
        return Eval(static_cast<std::size_t>(Root), Image);
    }

    // ===================== THE SAFETY RULE, AS CODE =====================
    //
    // An override can only ever remove a permission. This is the ONLY entry point
    // a permissive uses, and it is an AND — so there is no expression, however
    // written, that turns a FALSE into a TRUE.
    //
    // That is what makes an override from a downloaded track safe without anybody
    // reading it. Not "scripts are validated well": the operation is one that
    // cannot loosen anything.
    //
    // An expression that fails to parse or bind returns the permissive UNCHANGED
    // rather than FALSE. A broken override must not be able to stop a ride either
    // — that would make a typo a denial of service, and the failure is reported
    // through Error() where somebody can read it.
    bool Restrict(bool bPermissive, const FProcessImage& Image) const
    {
        if (!IsValid() || ResultType() != EPlcType::Bool)
        {
            return bPermissive;
        }
        return bPermissive && Evaluate(Image).AsBool();
    }

    static FPlcExpr Parse(const std::string& Source, const FProcessImage& Image);

private:
    FPlcValue Eval(std::size_t I, const FProcessImage& Image) const
    {
        const FNode& N = Node[I];
        switch (N.Op)
        {
        case EOp::Const: return N.Value;
        case EOp::Load:  return Image.At(N.Slot);
        case EOp::Neg:   return FPlcValue::Number(-Eval(Kid(N.A), Image).AsReal());
        case EOp::Not:   return FPlcValue::Boolean(!Eval(Kid(N.A), Image).AsBool());
        case EOp::Sel:
            // SEL(G, IN0, IN1): IN0 when G is FALSE. IEC 61131-3's order, and the
            // opposite way round from how most people read it — which is exactly
            // why it is spelled the standard's way rather than a friendlier one.
            return Eval(N.A >= 0 && Eval(Kid(N.A), Image).AsBool() ? Kid(N.C) : Kid(N.B), Image);
        default: break;
        }

        const FPlcValue L = Eval(Kid(N.A), Image);
        const FPlcValue R = Eval(Kid(N.B), Image);
        switch (N.Op)
        {
        case EOp::Add: return FPlcValue::Number(L.AsReal() + R.AsReal());
        case EOp::Sub: return FPlcValue::Number(L.AsReal() - R.AsReal());
        case EOp::Mul: return FPlcValue::Number(L.AsReal() * R.AsReal());
        case EOp::Div:
            // DIVISION BY ZERO IS ZERO, not a trap and not an infinity. An
            // override is evaluated inside a scan that must finish, so there is
            // nowhere for an exception to go; and a NaN propagating into a
            // permissive would make it neither true nor false. Zero is the
            // conservative answer because the result feeds an AND.
            return FPlcValue::Number(R.AsReal() != 0.0 ? L.AsReal() / R.AsReal() : 0.0);
        case EOp::Mod:
            return FPlcValue::Number(R.AsReal() != 0.0 ? std::fmod(L.AsReal(), R.AsReal()) : 0.0);
        case EOp::Lt: return FPlcValue::Boolean(L.AsReal() < R.AsReal());
        case EOp::Le: return FPlcValue::Boolean(L.AsReal() <= R.AsReal());
        case EOp::Gt: return FPlcValue::Boolean(L.AsReal() > R.AsReal());
        case EOp::Ge: return FPlcValue::Boolean(L.AsReal() >= R.AsReal());
        case EOp::Eq:
            return FPlcValue::Boolean(L.Type == EPlcType::Bool && R.Type == EPlcType::Bool
                ? L.bBool == R.bBool : L.AsReal() == R.AsReal());
        case EOp::Ne:
            return FPlcValue::Boolean(L.Type == EPlcType::Bool && R.Type == EPlcType::Bool
                ? L.bBool != R.bBool : L.AsReal() != R.AsReal());
        case EOp::And: return FPlcValue::Boolean(L.AsBool() && R.AsBool());
        case EOp::Or:  return FPlcValue::Boolean(L.AsBool() || R.AsBool());
        case EOp::Xor: return FPlcValue::Boolean(L.AsBool() != R.AsBool());
        default:       return FPlcValue::Boolean(false);
        }
    }

    static std::size_t Kid(int I) { return static_cast<std::size_t>(I); }

    friend class FPlcParser;
    std::vector<FNode> Node;
    int Root = -1;
    std::string Err;
    std::size_t ErrPos = 0;
};

// ------------------------------------------------------------------ lexer

class FPlcLexer
{
public:
    explicit FPlcLexer(const std::string& In) : Src(In) {}

    FTok Next()
    {
        SkipSpace();
        FTok T;
        T.At = P;
        if (P >= Src.size()) { return T; }

        const char C = Src[P];
        if (IsDigit(C) || (C == '.' && P + 1 < Src.size() && IsDigit(Src[P + 1])))
        {
            std::size_t Start = P;
            while (P < Src.size() && (IsDigit(Src[P]) || Src[P] == '.')) { ++P; }
            T.Kind = ETok::Number;
            T.Number = std::atof(Src.substr(Start, P - Start).c_str());
            return T;
        }
        if (IsAlpha(C))
        {
            // A NAME INCLUDES ITS SUBSCRIPT AND ITS FIELD. `block[3].clear` is one
            // identifier here rather than an index expression and a member access,
            // because the image is a flat set of named slots and there is no such
            // thing as a computed subscript. That is deliberate: `block[i]` where
            // i is a variable would need bounds checking at runtime and would make
            // binding impossible at parse time.
            std::size_t Start = P;
            while (P < Src.size() && (IsAlpha(Src[P]) || IsDigit(Src[P])
                   || Src[P] == '_' || Src[P] == '.' || Src[P] == '[' || Src[P] == ']'))
            {
                ++P;
            }
            T.Text = Src.substr(Start, P - Start);
            const std::string U = Upper(T.Text);
            if (U == "AND")        { T.Kind = ETok::And; }
            else if (U == "OR")    { T.Kind = ETok::Or; }
            else if (U == "XOR")   { T.Kind = ETok::Xor; }
            else if (U == "NOT")   { T.Kind = ETok::Not; }
            else if (U == "MOD")   { T.Kind = ETok::Mod; }
            else if (U == "TRUE")  { T.Kind = ETok::True; }
            else if (U == "FALSE") { T.Kind = ETok::False; }
            else                   { T.Kind = ETok::Name; }
            return T;
        }

        ++P;
        switch (C)
        {
        case '(': T.Kind = ETok::LParen; return T;
        case ')': T.Kind = ETok::RParen; return T;
        case ',': T.Kind = ETok::Comma;  return T;
        case '+': T.Kind = ETok::Plus;   return T;
        case '-': T.Kind = ETok::Minus;  return T;
        case '*': T.Kind = ETok::Star;   return T;
        case '/': T.Kind = ETok::Slash;  return T;
        case '=': T.Kind = ETok::Eq;     return T;   // ONE equals. ST, not C.
        case '<':
            if (P < Src.size() && Src[P] == '=') { ++P; T.Kind = ETok::Le; return T; }
            if (P < Src.size() && Src[P] == '>') { ++P; T.Kind = ETok::Ne; return T; }
            T.Kind = ETok::Lt; return T;
        case '>':
            if (P < Src.size() && Src[P] == '=') { ++P; T.Kind = ETok::Ge; return T; }
            T.Kind = ETok::Gt; return T;
        default: break;
        }
        T.Kind = ETok::End;
        T.Text = std::string(1, C);
        Bad = true;
        return T;
    }

    bool HadBadCharacter() const { return Bad; }
    static std::string Upper(const std::string& In)
    {
        std::string O = In;
        for (char& C : O) { if (C >= 'a' && C <= 'z') { C = static_cast<char>(C - 'a' + 'A'); } }
        return O;
    }

private:
    void SkipSpace()
    {
        while (P < Src.size())
        {
            // (* ST block comments *), and // to end of line, which real tools all
            // accept. An override with no way to say WHY it exists is an override
            // nobody dares change.
            if (Src[P] == ' ' || Src[P] == '\t' || Src[P] == '\n' || Src[P] == '\r') { ++P; }
            else if (P + 1 < Src.size() && Src[P] == '(' && Src[P + 1] == '*')
            {
                P += 2;
                while (P + 1 < Src.size() && !(Src[P] == '*' && Src[P + 1] == ')')) { ++P; }
                P = P + 1 < Src.size() ? P + 2 : Src.size();
            }
            else if (P + 1 < Src.size() && Src[P] == '/' && Src[P + 1] == '/')
            {
                while (P < Src.size() && Src[P] != '\n') { ++P; }
            }
            else { break; }
        }
    }
    static bool IsDigit(char C) { return C >= '0' && C <= '9'; }
    static bool IsAlpha(char C) { return (C >= 'a' && C <= 'z') || (C >= 'A' && C <= 'Z') || C == '_'; }

    const std::string& Src;
    std::size_t P = 0;
    bool Bad = false;
};

// ------------------------------------------------------------------ parser
//
// Precedence climbing, IEC 61131-3's table: OR lowest, then XOR, AND, comparison,
// additive, multiplicative, unary, primary.

class FPlcParser
{
public:
    FPlcParser(const std::string& In, const FProcessImage& InImage)
        : Lex(In), Image(InImage) {}

    FPlcExpr Run()
    {
        Advance();
        const int R = ParseOr();
        if (Out.Err.empty() && Tok.Kind != ETok::End)
        {
            Fail("unexpected trailing input");
        }
        if (Out.Err.empty() && Lex.HadBadCharacter())
        {
            Fail("unrecognised character");
        }
        Out.Root = Out.Err.empty() ? R : -1;
        return Out;
    }

private:
    void Advance() { Tok = Lex.Next(); }

    void Fail(const std::string& Why)
    {
        if (Out.Err.empty())
        {
            Out.Err = Why;
            Out.ErrPos = Tok.At;
        }
    }

    int Emit(const FNode& N)
    {
        Out.Node.push_back(N);
        return static_cast<int>(Out.Node.size()) - 1;
    }

    // A boolean operator applied to a REAL is a type error rather than a coercion.
    // ST is strongly typed and, more to the point, `zone[0].output AND ...` almost
    // certainly means the author wanted a comparison and forgot it.
    bool Want(int Child, EPlcType T, const char* What)
    {
        if (Child < 0) { return false; }
        if (Out.Node[static_cast<std::size_t>(Child)].Type != T)
        {
            Fail(std::string(What) + " needs a " + (T == EPlcType::Bool ? "BOOL" : "REAL"));
            return false;
        }
        return true;
    }

    int Binary(EOp Op, int L, int R, EPlcType In, EPlcType Result, const char* What)
    {
        if (!Want(L, In, What) || !Want(R, In, What)) { return -1; }
        FNode N;
        N.Op = Op; N.A = L; N.B = R; N.Type = Result;
        return Emit(N);
    }

    int ParseOr()
    {
        int L = ParseXor();
        while (Tok.Kind == ETok::Or)
        {
            Advance();
            L = Binary(EOp::Or, L, ParseXor(), EPlcType::Bool, EPlcType::Bool, "OR");
        }
        return L;
    }
    int ParseXor()
    {
        int L = ParseAnd();
        while (Tok.Kind == ETok::Xor)
        {
            Advance();
            L = Binary(EOp::Xor, L, ParseAnd(), EPlcType::Bool, EPlcType::Bool, "XOR");
        }
        return L;
    }
    int ParseAnd()
    {
        int L = ParseCompare();
        while (Tok.Kind == ETok::And)
        {
            Advance();
            L = Binary(EOp::And, L, ParseCompare(), EPlcType::Bool, EPlcType::Bool, "AND");
        }
        return L;
    }
    int ParseCompare()
    {
        int L = ParseAdd();
        while (Tok.Kind == ETok::Lt || Tok.Kind == ETok::Le || Tok.Kind == ETok::Gt
            || Tok.Kind == ETok::Ge || Tok.Kind == ETok::Eq || Tok.Kind == ETok::Ne)
        {
            const ETok K = Tok.Kind;
            Advance();
            const int R = ParseAdd();
            if (L < 0 || R < 0) { return -1; }
            // = and <> work on BOOL as well as REAL, because "is this lamp the
            // same as that one" is a real thing to ask. The ordered comparisons
            // do not: TRUE > FALSE is a C accident, not a question.
            const bool bOrdered = K == ETok::Lt || K == ETok::Le || K == ETok::Gt || K == ETok::Ge;
            if (bOrdered && !(Want(L, EPlcType::Real, "comparison")
                           && Want(R, EPlcType::Real, "comparison")))
            {
                return -1;
            }
            if (!bOrdered
                && Out.Node[static_cast<std::size_t>(L)].Type
                != Out.Node[static_cast<std::size_t>(R)].Type)
            {
                Fail("= and <> compare two BOOLs or two REALs, not one of each");
                return -1;
            }
            FNode N;
            N.Op = K == ETok::Lt ? EOp::Lt : K == ETok::Le ? EOp::Le
                 : K == ETok::Gt ? EOp::Gt : K == ETok::Ge ? EOp::Ge
                 : K == ETok::Eq ? EOp::Eq : EOp::Ne;
            N.A = L; N.B = R; N.Type = EPlcType::Bool;
            L = Emit(N);
        }
        return L;
    }
    int ParseAdd()
    {
        int L = ParseMul();
        while (Tok.Kind == ETok::Plus || Tok.Kind == ETok::Minus)
        {
            const EOp Op = Tok.Kind == ETok::Plus ? EOp::Add : EOp::Sub;
            Advance();
            L = Binary(Op, L, ParseMul(), EPlcType::Real, EPlcType::Real, "arithmetic");
        }
        return L;
    }
    int ParseMul()
    {
        int L = ParseUnary();
        while (Tok.Kind == ETok::Star || Tok.Kind == ETok::Slash || Tok.Kind == ETok::Mod)
        {
            const EOp Op = Tok.Kind == ETok::Star ? EOp::Mul
                         : Tok.Kind == ETok::Slash ? EOp::Div : EOp::Mod;
            Advance();
            L = Binary(Op, L, ParseUnary(), EPlcType::Real, EPlcType::Real, "arithmetic");
        }
        return L;
    }
    int ParseUnary()
    {
        if (Tok.Kind == ETok::Not)
        {
            Advance();
            const int A = ParseUnary();
            if (!Want(A, EPlcType::Bool, "NOT")) { return -1; }
            FNode N; N.Op = EOp::Not; N.A = A; N.Type = EPlcType::Bool;
            return Emit(N);
        }
        if (Tok.Kind == ETok::Minus)
        {
            Advance();
            const int A = ParseUnary();
            if (!Want(A, EPlcType::Real, "negation")) { return -1; }
            FNode N; N.Op = EOp::Neg; N.A = A; N.Type = EPlcType::Real;
            return Emit(N);
        }
        if (Tok.Kind == ETok::Plus) { Advance(); return ParseUnary(); }
        return ParsePrimary();
    }
    int ParsePrimary()
    {
        switch (Tok.Kind)
        {
        case ETok::Number:
        {
            FNode N; N.Op = EOp::Const; N.Value = FPlcValue::Number(Tok.Number);
            N.Type = EPlcType::Real;
            Advance();
            return Emit(N);
        }
        case ETok::True:
        case ETok::False:
        {
            FNode N; N.Op = EOp::Const;
            N.Value = FPlcValue::Boolean(Tok.Kind == ETok::True);
            N.Type = EPlcType::Bool;
            Advance();
            return Emit(N);
        }
        case ETok::LParen:
        {
            Advance();
            const int A = ParseOr();
            if (Tok.Kind != ETok::RParen) { Fail("expected )"); return -1; }
            Advance();
            return A;
        }
        case ETok::Name:
            return ParseNameOrCall();
        default:
            Fail("expected a value");
            return -1;
        }
    }

    int ParseNameOrCall()
    {
        const std::string Ident = Tok.Text;
        const std::string U = FPlcLexer::Upper(Ident);
        Advance();

        if (Tok.Kind == ETok::LParen)
        {
            return ParseCall(U);
        }

        // BINDING HAPPENS HERE, at parse time, against a specific image shape. An
        // override written for a layout with eight blocks fails to load on one
        // with six rather than reading a slot that now means something else — the
        // same argument as the PLC refusing to RUN a program built for another
        // track.
        std::size_t Slot = 0;
        EPlcType T = EPlcType::Bool;
        if (!Image.Find(Ident, Slot, T))
        {
            Fail("no such name in the process image: " + Ident);
            return -1;
        }
        FNode N; N.Op = EOp::Load; N.Slot = Slot; N.Type = T;
        return Emit(N);
    }

    int ParseCall(const std::string& Name)
    {
        Advance();                          // past (
        std::vector<int> Args;
        if (Tok.Kind != ETok::RParen)
        {
            for (;;)
            {
                Args.push_back(ParseOr());
                if (Tok.Kind != ETok::Comma) { break; }
                Advance();
            }
        }
        if (Tok.Kind != ETok::RParen) { Fail("expected ) after arguments"); return -1; }
        Advance();
        for (int A : Args) { if (A < 0) { return -1; } }

        if (Name == "SEL")
        {
            // SEL(G, IN0, IN1) — IN0 when G is FALSE. IEC 61131-3's order, kept
            // even though it reads backwards to most people, because a subset that
            // reversed it would teach the wrong thing.
            if (Args.size() != 3) { Fail("SEL takes three arguments: SEL(G, IN0, IN1)"); return -1; }
            if (!Want(Args[0], EPlcType::Bool, "SEL's first argument")) { return -1; }
            const EPlcType T0 = Out.Node[static_cast<std::size_t>(Args[1])].Type;
            const EPlcType T1 = Out.Node[static_cast<std::size_t>(Args[2])].Type;
            if (T0 != T1) { Fail("SEL's two results must be the same type"); return -1; }
            FNode N; N.Op = EOp::Sel; N.A = Args[0]; N.B = Args[1]; N.C = Args[2]; N.Type = T0;
            return Emit(N);
        }
        if (Name == "MIN" || Name == "MAX")
        {
            if (Args.size() != 2) { Fail(Name + " takes two arguments"); return -1; }
            if (!Want(Args[0], EPlcType::Real, Name.c_str())
                || !Want(Args[1], EPlcType::Real, Name.c_str())) { return -1; }
            // Built from SEL and a comparison rather than as new opcodes, because
            // two fewer cases in the evaluator is two fewer places to be wrong.
            FNode Cmp; Cmp.Op = Name == "MIN" ? EOp::Lt : EOp::Gt;
            Cmp.A = Args[0]; Cmp.B = Args[1]; Cmp.Type = EPlcType::Bool;
            const int G = Emit(Cmp);
            FNode N; N.Op = EOp::Sel; N.A = G; N.B = Args[1]; N.C = Args[0];
            N.Type = EPlcType::Real;
            return Emit(N);
        }
        Fail("no such function: " + Name);
        return -1;
    }

    FPlcLexer Lex;
    const FProcessImage& Image;
    FTok Tok;
    FPlcExpr Out;
};

inline FPlcExpr FPlcExpr::Parse(const std::string& Source, const FProcessImage& Image)
{
    FPlcParser P(Source, Image);
    return P.Run();
}

// ponytail: no ABS, SQRT, LIMIT, or the rest of IEC 61131-3's standard functions.
// SEL, MIN and MAX are the three a permissive has actually wanted so far; the
// others are one case each in ParseCall the day something needs them, and adding
// them speculatively would be a standard library with no callers.
//
// Also no constant folding. An override is a handful of nodes evaluated once per
// scan; folding it would be optimising something nothing has measured.

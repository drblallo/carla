// Declares clang::SyntaxOnlyAction.
#include "clang/Frontend/FrontendActions.h"
#include "clang/Tooling/CommonOptionsParser.h"
#include "clang/Tooling/Tooling.h"
// Declares llvm::cl::extrahelp.
#include "clang/ASTMatchers/ASTMatchFinder.h"
#include "clang/ASTMatchers/ASTMatchers.h"
#include "llvm/Support/CommandLine.h"

using namespace clang;
using namespace clang::ast_matchers;

static std::string getSourceText(SourceRange SR, SourceManager &SM) {
  // Using Lexer::getSourceText to extract text from SourceRange
  return std::string(Lexer::getSourceText(CharSourceRange::getTokenRange(SR),
                                          SM, LangOptions()));
}

StatementMatcher EnumMatcher =
    cxxMemberCallExpr(
        allOf(hasDescendant(cxxConstructExpr(has(
                  implicitCastExpr(has(stringLiteral().bind("python_name")))))),
              callee(cxxMethodDecl(hasName("value"))),
              hasType(cxxRecordDecl(hasName("enum_"))),
              forEach(expr(allOf(
                  unless(memberExpr()),
                  forEachDescendant(stringLiteral().bind("entry_name")))))))
        .bind("decl");

StatementMatcher ClassMatcher =
    cxxTemporaryObjectExpr(
        allOf(hasType(cxxRecordDecl(hasName("class_"))),
              hasDescendant(stringLiteral().bind("python_name"))))
        .bind("decl");

StatementMatcher FunctionMatcher =
    cxxMemberCallExpr(
        allOf(
            callee(cxxMethodDecl(anyOf(hasName("def"), hasName("add_property"),
                                       hasName("def_readwrite")))),
            hasDescendant(
                expr(hasType(cxxRecordDecl(hasName("class_")))).bind("class")),
            has(implicitCastExpr(has(stringLiteral().bind("python_name")))),
            optionally(forEach(
                unaryOperator(hasDescendant(lambdaExpr())).bind("lambda"))),
            optionally(forEach(
                materializeTemporaryExpr(has(implicitCastExpr(has(unaryOperator(
                    hasDescendant(declRefExpr().bind("member_access"))))))))),
            optionally(forEach(
                expr(allOf(unless(memberExpr()),
                           forEachDescendant(cxxConstructExpr(hasDescendant(
                               stringLiteral().bind("arg"))))))))))
        .bind("call");

static std::string typeToString(clang::QualType t) {
  std::string s;
  llvm::raw_string_ostream OS(s);
  LangOptions ops;
  PrintingPolicy policy(ops);
  policy.PrintCanonicalTypes = 1;
  policy.FullyQualifiedName = 1;
  policy.SuppressScope = 0;
  policy.Bool = 1;
  policy.AlwaysIncludeTypeForTemplateArgument = 1;
  t.getNonReferenceType().print(OS, policy);
  OS.flush();
  return s;
}

template <typename T> static std::string toString(T *astNode) {
  LangOptions opts;
  std::string s;
  llvm::raw_string_ostream OS(s);
  PrintingPolicy policy(opts);
  policy.PrintCanonicalTypes = 1;
  policy.FullyQualifiedName = 1;
  policy.SuppressScope = 0;
  policy.Bool = 1;
  policy.AlwaysIncludeTypeForTemplateArgument = 1;
  astNode->print(OS, policy);
  OS.flush();
  return s;
}

template <typename T>
static std::string toStringPretty(T *astNode, bool FullyQualifiedName = true) {
  LangOptions opts;
  std::string s;
  llvm::raw_string_ostream OS(s);
  PrintingPolicy policy(opts);
  if (FullyQualifiedName) {
    policy.PrintCanonicalTypes = 1;
    policy.FullyQualifiedName = 1;
  } else {
    policy.PrintCanonicalTypes = 0;
    policy.FullyQualifiedName = 0;
  }
  policy.SuppressScope = 0;
  policy.Bool = 1;
  policy.AlwaysIncludeTypeForTemplateArgument = 1;
  astNode->printPretty(OS, nullptr, policy);
  OS.flush();
  return s;
}

class EnumPrinter : public MatchFinder::MatchCallback {
public:
  virtual void run(const MatchFinder::MatchResult &Result) {
    auto *var = Result.Nodes.getNodeAs<CXXMemberCallExpr>("decl");
    auto *enumValue = llvm::dyn_cast<DeclRefExpr>(var->getArg(1));

    auto *enumDecl = Result.Nodes.getNodeAs<StringLiteral>("python_name");
    auto *entryName = Result.Nodes.getNodeAs<StringLiteral>("entry_name");

    std::string enumName = enumDecl->getString().str();
    if (enumValue != nullptr) {
      std::string entry = toString(enumValue->getReferencedDeclOfCallee());
      enumToEntries[enumName].insert(enumToEntries[enumName].begin(), entry);
      // enumToUnderlyingType[enumName] = typeToString(enumValue->getType());
    } else {
      std::string entry = getSourceText(var->getArg(1)->getSourceRange(),
                                        *Result.SourceManager);
      enumToEntries[enumName].insert(enumToEntries[enumName].begin(), entry);
    }
  }

  std::map<std::string, std::vector<std::string>> enumToEntries;
  std::map<std::string, std::string> enumToUnderlyingType;
};

class ClassPrinter : public MatchFinder::MatchCallback {
public:
  virtual void run(const MatchFinder::MatchResult &Result) {
    // Result.Nodes.getNodeAs<CXXTemporaryObjectExpr>("decl")->dump();
    auto *var = Result.Nodes.getNodeAs<CXXTemporaryObjectExpr>("decl");
    auto arg = var->getType()
                   ->castAs<ElaboratedType>()
                   ->getNamedType()
                   ->castAs<TemplateSpecializationType>()
                   ->template_arguments()[0]
                   .getAsType();
    llvm::outs()
        << "class RLC_"
        << Result.Nodes.getNodeAs<StringLiteral>("python_name")->getString()
        << "{ char payload" << Result.Context->getTypeInfo(arg).Width
        << ";};\n";
  }
};

class FunctionPrinter : public MatchFinder::MatchCallback {
public:
  void registerSignature(clang::QualType casterType,
                         clang::QualType functionType,
                         const std::string &pythonMethodName, const Expr *key) {
    assert(functionType->isFunctionType());
    auto *castedType = llvm::cast<FunctionType>(functionType);

    methodToReturnType[key] = typeToString(castedType->getReturnType());

    assert(castedType->isFunctionProtoType());
    auto *functionProtType = llvm::cast<FunctionProtoType>(castedType);
    methodToArgTypes[key].clear();

    if (casterType->isMemberFunctionPointerType()) {
      auto memberPointerType = llvm::cast<MemberPointerType>(casterType);
      methodToArgTypes[key].push_back(
          typeToString(memberPointerType->getClass()
                           ->getLocallyUnqualifiedSingleStepDesugaredType()));
      isMemberExpr.insert(key);
    }

    for (auto arg : functionProtType->param_types()) {

      methodToArgTypes[key].push_back(typeToString(arg));
    }
  }

  virtual void run(const MatchFinder::MatchResult &Result) {
    //
    auto argName = Result.Nodes.getNodeAs<StringLiteral>("arg");
    auto call = Result.Nodes.getNodeAs<CXXMemberCallExpr>("call");
    const clang::Expr *key = call;

    auto pythonMethodName =
        Result.Nodes.getNodeAs<StringLiteral>("python_name")->getString();
    methodToName[key] = pythonMethodName;

    auto &entry = methodToArgs[key];
    if (argName != nullptr)
      entry.push_back(argName->getString().str());

    if (call->getNumArgs() < 1) {
      assert(false);
      return;
    }

    auto implicitConversion = llvm::dyn_cast<UnaryOperator>(call->getArg(1));
    if (!implicitConversion) {

      // lambda that returns a member access
      auto access = Result.Nodes.getNodeAs<DeclRefExpr>("member_access");
      if (access) {
        methodToCPPMethod[key] = "[](auto& arg){ return &arg." +
                                 access->getNameInfo().getAsString() + ";}";
        methodToReturnType[key] = typeToString(access->getType()) + "*";
        methodToArgTypes[key].push_back(
            typeToString(access->getQualifier()
                             ->getAsType()
                             ->getLocallyUnqualifiedSingleStepDesugaredType()));

        return;
      }
      // ToDo, actually handle this stuff
      std::string namespaces;
      const Stmt *expr = call->getArg(1);
      const Decl *decl = nullptr;
      while (expr != nullptr)
        for (auto parent : Result.Context->getParents(*expr)) {
          expr = parent.get<Stmt>();
          decl = parent.get<Decl>();
          break;
        }

      while (decl != nullptr and not Result.Context->getParents(*decl).empty())
        for (auto parent : Result.Context->getParents(*decl)) {
          if (const NamespaceDecl *ns = parent.get<NamespaceDecl>()) {
            namespaces = "::" + ns->getNameAsString() + namespaces;
          }
          decl = parent.get<Decl>() == decl ? nullptr : parent.get<Decl>();
          break;
        }

      extraUsings[key] = namespaces;
      // methodToCPPMethod[key] = toStringPretty(call->getArg(1), false);
      methodToCPPMethod[key] = getSourceText(call->getArg(1)->getSourceRange(),
                                             *Result.SourceManager);
      // methodToReturnType[key] = typeToString(call->getArg(1)->getType());
      methodToReturnType[key] = "auto";

      // auto classType = Result.Nodes.getNodeAs<Expr>("class")->getType();

      // auto typeNameWithBoostPythonWrapper = typeToString(classType);
      //  methodToArgTypes[key].push_back(typeNameWithBoostPythonWrapper.substr(
      //  28, typeNameWithBoostPythonWrapper.size() - 29));
      return;
    }

    if (auto function =
            llvm::dyn_cast<DeclRefExpr>(implicitConversion->getSubExpr())) {
      if (auto fieldDecl = llvm::dyn_cast<FieldDecl>(function->getDecl())) {
        methodToCPPMethod[key] =
            "[](auto& arg){ return &arg." + fieldDecl->getName().str() + ";}";
        methodToReturnType[key] = typeToString(fieldDecl->getType()) + "*";
        methodToArgTypes[key].push_back(
            typeToString(QualType(fieldDecl->getParent()->getTypeForDecl(), 0)));
        return;
      } else {
        methodToCPPMethod[key] =
            function->getDecl()->getQualifiedNameAsString();
        registerSignature(implicitConversion->getType(),
                          function->getDecl()->getType(),
                          pythonMethodName.str(), key);
        return;
      }
    } else {
      auto lambda = Result.Nodes.getNodeAs<Expr>("lambda");
      assert(lambda != nullptr);
      methodToCPPMethod[key] =
          "(" + getSourceText(lambda->getSourceRange(), *Result.SourceManager) +
          ")";
      registerSignature(
          implicitConversion->getType(),
          lambda->getType()->castAs<PointerType>()->getPointeeType(),
          pythonMethodName.str(), key);
      return;
    }
    assert(false);
  }

  std::set<const clang::Expr *> isMemberExpr;
  std::map<const clang::Expr *, std::string> methodToName;
  std::map<const clang::Expr *, std::string> extraUsings;
  std::map<const clang::Expr *, std::vector<std::string>> methodToArgs;
  std::map<const clang::Expr *, std::string> methodToCPPMethod;
  std::map<const clang::Expr *, std::string> methodToReturnType;
  std::map<const clang::Expr *, std::vector<std::string>> methodToArgTypes;
};

using namespace clang::tooling;
using namespace llvm;

// Apply a custom category to all command-line options so that they are the
// only ones displayed.
static cl::OptionCategory MyToolCategory("my-tool options");

// CommonOptionsParser declares HelpMessage with a description of the common
// command-line options related to the compilation database and input files.
// It's nice to have this help message in all tools.
static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

// A help message for this specific tool can be added afterwards.
static cl::extrahelp MoreHelp("\nMore help text...\n");

static bool is_number(const std::string &s) {
  return !s.empty() && std::all_of(s.begin(), s.end(), ::isdigit);
}

int main(int argc, const char **argv) {
  auto ExpectedParser = CommonOptionsParser::create(argc, argv, MyToolCategory);
  if (!ExpectedParser) {
    llvm::errs() << ExpectedParser.takeError();
    return 1;
  }
  CommonOptionsParser &OptionsParser = ExpectedParser.get();
  ClangTool Tool(OptionsParser.getCompilations(),
                 OptionsParser.getSourcePathList());
  MatchFinder Finder;

  ClassPrinter Printer;
  Finder.addMatcher(ClassMatcher, &Printer);

  FunctionPrinter Printer2;
  Finder.addMatcher(FunctionMatcher, &Printer2);

  EnumPrinter enumPrinter;
  Finder.addMatcher(EnumMatcher, &enumPrinter);
  auto result = Tool.run(newFrontendActionFactory(&Finder).get());

  for (auto &pair : enumPrinter.enumToEntries) {
    llvm::outs() << "enum class " << pair.first << ": " << "int64_t" << "{\n";
    for (auto &entry : pair.second) {
      llvm::outs() << " " << (is_number(entry) ? "_" + entry : entry) << ",\n";
    }
    llvm::outs() << "};\n";
  }

  size_t i = 0;
  for (auto &entry : Printer2.methodToArgs) {
    // replace the name of the method with the real first argumetn
    if (Printer2.methodToArgTypes[entry.first].size() != 0)
      entry.second.insert(entry.second.begin(), "self");
    auto &argTypes = Printer2.methodToArgTypes[entry.first];

    size_t unnamedArgsId = 0;
    while (entry.second.size() < argTypes.size()) {
      entry.second.push_back(
          ("unnammed_arg_" + llvm::Twine(unnamedArgsId++)).str());
    }

    auto &returnType = Printer2.methodToReturnType[entry.first];
    llvm::outs() << (returnType != "" ? returnType : "MISSING") << " " << "rlc_"
                 << Printer2.methodToName[entry.first] << i++ << "(";

    size_t i = 0;
    for (auto [name, argType] : llvm::zip_longest(entry.second, argTypes)) {
      llvm::outs() << argType << "* " << name;
      i++;
      if (i != std::max(argTypes.size(), entry.second.size()))
        llvm::outs() << ", ";
    }

    auto &cppName = Printer2.methodToCPPMethod[entry.first];
    llvm::outs() << ") {" << "\n  ";
    if (auto iter = Printer2.extraUsings.find(entry.first);
        iter != Printer2.extraUsings.end() and not iter->second.empty()) {

      llvm::outs() << "  using namespace " << Printer2.extraUsings[entry.first]
                   << ";\n";
    }
    llvm::outs() << (returnType != "void" ? "return " : "");
    if (Printer2.isMemberExpr.count(entry.first)) {
      llvm::outs() << "self->";
    }
    llvm::outs() << (cppName != "" ? cppName : "MISSING") << "(";
    i = Printer2.isMemberExpr.count(entry.first);
    for (auto &arg : llvm::drop_begin(
             entry.second, Printer2.isMemberExpr.count(entry.first))) {
      llvm::outs() << "*" << arg;
      i++;
      if (i != entry.second.size())
        llvm::outs() << ", ";
    }
    llvm::outs() << ");\n}\n\n";
  }

  return result;
}

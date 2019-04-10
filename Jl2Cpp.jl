
module Jl2Cpp
    DLL_EXPORT = "#ifdef _MSC_VER\n__declspec(dllexport)\n#endif\nextern \"C\""
    #CC = "g++"
    CC = haskey(ENV, "JL2CPP_CC")? ENV["JL2CPP_CC"]: "D:\\LLVM\\bin\\clang++.exe"
    #C_OPT = ["-std=c++14", """-I$(joinpath(Base.source_dir(), "includes"))""", "-shared", "-Ofast", "-g"]
    C_OPT = haskey(ENV, "JL2CPP_C_OPT")? [split(ENV["JL2CPP_C_OPT"])..., """-I$(joinpath(Base.source_dir(), "includes"))"""]: ["-std=c++14", """-I$(joinpath(Base.source_dir(), "includes"))""", "-shared", "-Ofast", "-g"]
    CPP_PREDEFS = []
    CPP_INCLUDES = ["#include<cstdint>", "#include<tuple>", "#include<algorithm>", "#include<complex>", "#include<cstdio>", "#include<cfenv>", "#include<cassert>", "#include<jl2cpp.hpp>", "#include<Eigen/Dense>"]
    CPP_USINGS = ["using namespace Eigen;"]

    """
    Type to pass matrixes or arrays to C++ function
    """
    mutable struct CMatrix{T}
        ptr::Ptr{T}
        rsize::Csize_t
        csize::Csize_t
    end
    export CMatrix
    mutable struct CTuple
        ptr::Ptr{Void}
        size::Csize_t
    end
    export CTuple

    function ConvertToArray{T}(m::CMatrix{T})
        # ret = (m.csize == 1)? Array{T, 1}(m.rsize): Array{T, 2}(m.rsize, m.csize)
        unsafe_wrap((m.csize == 1)? Array{T, 1}: Array{T, 2}, m.ptr, (m.csize == 1)?m.rsize:(m.rsize, m.csize), false)
    end
    export ConvertToArray

    function ConvertToTuple(ty::DataType, from::CTuple)
        assert(ty.name.name == :Tuple)
        ret = :((1,))
        ret.args = map(x->:(zero($x)), ty.parameters)
        ret = eval(ret)
        offset = 0
        for i=1:length(ret)
            unsafe_copy!(
                convert(Ptr{fieldtype(ty, i)}, pointer_from_objref(ret) + fieldoffset(ty, i)),
                convert(Ptr{fieldtype(ty, i)}, from.ptr + offset),
                1
            )
            offset += sizeof(fieldtype(ty, i))
            if offset > from.size; error("wrong tuple size"); end
        end
        ret
    end
    export ConvertToTuple

    mutable struct GenCppCodeInfo
        indent :: Int64
        userDefFunc::Set{String}
        argArrays::Set{Symbol}  # arrays of user-defined function are safe to get pointer, so use them to speedup
    end
    GenCppCodeInfo() = GenCppCodeInfo(0, Set([]), Set([]))

    function putInfo(str)
        if !haskey(ENV, "JL2CPP_QUIET")
            println(str)
        end
    end
    function IsType(x)
        if x isa DataType; return true; end
        if x isa Expr && x.head == :call && x.args[1] in [:eltype]; return true; end # for functions that return type
        try
            x isa Symbol && eval(x) isa DataType
        catch
            false
        end
    end
    function GetType(x, info)
        if x isa DataType; return ConvertType(x); end
        if x isa Expr && x.head == :call && x.args[1] in [:eltype]; return GenerateCppCode(x, info); end # for functions that return type
        if x isa Symbol; return ConvertType(eval(x)); end
    end
    RECIPES = []
    function __init__()
        for idx in find(x->x=="-r", ARGS)
            rn = ARGS[idx + 1]
            putInfo("recipe: $rn")
            push!(RECIPES, evalfile(joinpath(@Base.__DIR__, "recipe", rn * ".jl")))
        end
    end

    function GenerateWrapperFunction(ex::Expr, info)::String
        if ex.args[1].head != :(::)
            error("functions must be typed")
        end
        name = ex.args[1].args[1].args[1]
        args = ex.args[1].args[1].args[2:end]
        retty = ex.args[1].args[2] |> eval
        rettycpp = if retty.name.name == :Array
            "jl2cpp::matrix"
        elseif retty.name.name == :Tuple
            "jl2cpp::tuple"
        else
            ConvertType(retty)
        end
        argstrs = String[]
        initstrs = String[]
        argnames = String[]
        info.argArrays = Set([])
        push!(initstrs, "initParallel();")
        for a = args
            if typeof(a) != Expr || a.head != :(::)
                error("function arguments must be typed")
            end
            cname = a.args[1]
            push!(argnames, cname)
            ty = eval(a.args[2])
            if ty.name.name == :Array
                cty = ty.parameters[1] |> ConvertType
                if ty.parameters[2] == 1
                    push!(argstrs, "jl2cpp::matrix *$(cname)_jl2cpp_arg")
                    push!(initstrs, repeat("\t", info.indent + 1) * "Map<Matrix<$cty, Dynamic, 1> > $cname(($cty *)$(cname)_jl2cpp_arg->ptr, $(cname)_jl2cpp_arg->rsize);")
                    #push!(initstrs, repeat("\t", info.indent + 1) * "$cty *$(cname)_jl2cpp_ptr = ($cty *)$(cname)_jl2cpp_arg->ptr;")
                    #push!(initstrs, repeat("\t", info.indent + 1) * "printf(\"size: %d\\n\", $(cname)_jl2cpp_arg->rsize);")
                elseif ty.parameters[2] == 2
                    push!(argstrs, "jl2cpp::matrix *$(cname)_jl2cpp_arg")
                    push!(initstrs, repeat("\t", info.indent + 1) * "Map<Matrix<$cty, Dynamic, Dynamic> > $cname(($cty *)$(cname)_jl2cpp_arg->ptr, $(cname)_jl2cpp_arg->rsize, $(cname)_jl2cpp_arg->csize);")
                    #push!(initstrs, repeat("\t", info.indent + 1) * "$cty *$(cname)_jl2cpp_ptr = ($cty *)$(cname)_jl2cpp_arg->ptr;")
                else
                    error("3+ dimension array is currently not supported.")
                end
            else
                cty = ty |> ConvertType
                push!(argstrs, "$cty $cname")
            end
        end
        argstr = join(argstrs, ", ")
        initstr = join(initstrs, "\n")
        retstr = ""
        if retty.name.name == :Void
            retstr = "$name($(join(argnames, ", ")));"
        elseif retty.name.name == :Tuple
            retstr = "auto jl2cpp_ret_tmp = $name($(join(argnames, ", ")));\n"
            retstr *= repeat("\t", info.indent + 1) * "void* jl2cpp_ret_ptr = malloc(jl2cpp::SumTupleSize<decltype(jl2cpp_ret_tmp)>::value);\n"
            retstr *= repeat("\t", info.indent + 1) * "std::size_t jl2cpp_ret_offset = 0;\n"
            i = 0
            for t in retty.parameters
                tcppty = ConvertType(t)
                retstr *= repeat("\t", info.indent + 1) * "*($tcppty *)((std::uint8_t *)jl2cpp_ret_ptr + jl2cpp_ret_offset) = std::get<$i>(jl2cpp_ret_tmp);\n"
                retstr *= repeat("\t", info.indent + 1) * "jl2cpp_ret_offset += sizeof($tcppty);\n"
                i += 1
            end
            retstr *= repeat("\t", info.indent + 1) * "return jl2cpp::tuple{jl2cpp_ret_ptr, jl2cpp_ret_offset};\n"
        else
            retstr = "return jl2cpp::calc_retval($name($(join(argnames, ", "))));"
        end

        body = "{\n" * initstr * (isempty(initstrs)? "": "\n") * repeat("\t", info.indent + 1) * retstr * "\n}"
        "$DLL_EXPORT $rettycpp $(name)_jl2cpp_wrapper($argstr)\n$body\n"
    end


    """
    Convert Julia type into C++ type.
    """
    function ConvertType(ty :: DataType)
        sym = ty.name.name
        if sym == :Int64
            "int64_t"
        elseif sym == :Int32
            "int32_t"
        elseif sym == :UInt64
            "uint64_t"
        elseif sym == :UInt32
            "uint32_t"
        elseif sym == :String
            "std::string"
        elseif sym == :Float64
            "double"
        elseif sym == :Float32
            "float"
        elseif sym == :Complex64
            "std::complex<float>"
        elseif sym == :Complex128
            "std::complex<double>"
        elseif sym == :Complex
            t = ConvertType(ty.parameters[1])
            "std::complex<$t>"
        elseif sym == :Void
            "void"
        elseif sym == :Tuple
            "std::tuple<"* join(map(x->if x.name.name in [:Array, :Tuple]; error("nested tuple is not supported");else; ConvertType(x);end, ty.parameters), ", ") * ">"
        elseif sym == :Array
            if ty.parameters[1] in [:Tuple, :Array]; error("nested array is not supported");end
            t = ConvertType(ty.parameters[1])
            if ty.parameters[2] == 1
                "Matrix<$t, Dynamic, 1>"
            elseif ty.parameters[2] == 2
                "Matrix<$t, Dynamic, Dynamic>"
            else
                error("3+ dimension array is not supported.")
            end
        elseif sym == :Char
            "char"
        else
            error("unknown type: $sym")
        end
    end

    """
    Convert Julia function into C++ form
    """
    function ConvertFunction(name :: String, args, info) :: String
        if name == "length"
            "$(GenerateCppCode(args[1], info)).size()"
        elseif name == "size"
            if length(args) != 2
                "std::forward_as_tuple($(GenerateCppCode(args[1], info)).rows(), $(GenerateCppCode(args[1], info)).cols())"
            elseif contains(GenerateCppCode(args[2], info), "1")
                "$(GenerateCppCode(args[1], info)).rows()"
            elseif contains(GenerateCppCode(args[2], info), "2")
                "$(GenerateCppCode(args[1], info)).cols()"
            else
                error("3+ dimension array is not supported.")
            end
        elseif name == "eye"
            "MatrixXd::Identity($(GenerateCppCode(args[1], info)), $(length(args) < 2? 1: GenerateCppCode(args[2], info))).eval()"
        elseif name == "zeros"
            if IsType(args[1])
                a = GetType(args[1], info)
                "Matrix<$a, Dynamic, Dynamic>::Zero($(GenerateCppCode(args[2], info)), $(length(args) < 3? 1: GenerateCppCode(args[3], info))).eval()"
            else
                a = GenerateCppCode(args[1], info)
                "MatrixXd::Zero($a, $(length(args) < 2? 1: GenerateCppCode(args[2], info))).eval()"
            end
        elseif name == "ones"
            if IsType(args[1])
                a = GetType(args[1], info)
                "Matrix<$a, Dynamic, Dynamic>::Ones($(GenerateCppCode(args[2], info)), $(length(args) < 3? 1: GenerateCppCode(args[3], info))).eval()"
            else
                a = GenerateCppCode(args[1], info)
                "MatrixXd::Ones($a, $(length(args) < 2? 1: GenerateCppCode(args[2], info))).eval()"
            end
        elseif name == "rand"
            #"(MatrixXd::Random($(GenerateCppCode(args[1], info)), $(GenerateCppCode(args[2], info))) + 1) / 2"
            # since arguments of `rand` is much complicated, the implementation is WIP
            if length(args) == 0
                "Matrix<double, 1, 1>::Random()[0]"
            elseif args[1] |>IsType
                typ = GetType(args[1], info)
                if args[1] in [:Float, :Float64, :Float32, :Float16, :Complex, :Complex128, :Complex64, :Complex32]
                    "((Matrix<$typ, 1, 1>::Random()[0] + ($typ)(1.0)) / 2)"
                else
                    "Matrix<$typ, 1, 1>::Random()[0]"
                end
            elseif isa(args[1], Expr) && args[1].head == :(:)
                error("not implemented yet")
            else
                # all arguments assumed dimension parameter
                if length(args) == 1
                    "jl2cpp::random($(GenerateCppCode(args[1], info)))"
                elseif length(args) == 2
                    "jl2cpp::random($(GenerateCppCode(args[1], info)),$(GenerateCppCode(args[2], info)))"
                else
                    error("3+ dimension is currently not supported")
                end
            end 
        elseif name == "randn"
            "jl2cpp::randn($(join(map(x->GenerateCppCode(x, info), args), ", ")))" # TODO: implement type argument
        elseif name == "mean"
            "$(GenerateCppCode(args[1], info)).mean()"
        elseif name == "trace"
            "$(GenerateCppCode(args[1], info)).trace()"
        elseif name == "std"
            "jl2cpp::stdDev($(GenerateCppCode(args[1], info)))"
        elseif name == "sum"
            "$(GenerateCppCode(args[1], info)).sum()"
        elseif name == "eltype"
            "typename jl2cpp::ElType<decltype($(GenerateCppCode(args[1], info)))>::type::Scalar"
        elseif name == "zero"
            if IsType(args[1])
                a = GetType(args[1], info)
                "static_cast<$a>(0)"
            else
                a = GenerateCppCode(args[1], info)
                "static_cast<jl2cpp::ElType<decltype($a)>::type>(0)"
            end
        elseif name == "one"
            if IsType(args[1])
                a = GetType(args[1], info)
                "static_cast<$a>(1)"
            else
                a = GenerateCppCode(args[1], info)
                "static_cast<jl2cpp::ElType<decltype($a)>::type>(1)"
            end
        elseif name == "hex"
            "jl2cpp::hex($(GenerateCppCode(args[1], info)))"
        elseif name == "parse"
            if length(args) < 1 || !IsType(args[1])
                error("parsing julia expression is not supported")
            end
            if length(args) < 2
                error("too few argument in parse")
            end
            typ = args[1]|>eval|>ConvertType
            base = length(args) == 3? GenerateCppCode(args[3], info): 10
            "jl2cpp::parse<$typ>($(GenerateCppCode(args[2], info)), $base)"
        elseif name == "max"
            if length(args) == 2
                argstr = map(x->GenerateCppCode(x, info), args)
                "($(argstr[1]) > $(argstr[2])? $(argstr[1]): $(argstr[2]))"
            else
                "(std::max)({$(join(map(x->GenerateCppCode(x, info), args), ", "))})"
            end
        elseif name == "min"
            if length(args) == 2
                argstr = map(x->GenerateCppCode(x, info), args)
                "($(argstr[1]) < $(argstr[2])? $(argstr[1]): $(argstr[2]))"
            else
                "(std::min)({$(join(map(x->GenerateCppCode(x, info), args), ", "))})"
            end
        # unary math functions defined in jl2cpp.hpp
        elseif name in ["abs", "sqrt", "log", "log10", "exp", "sin", "cos", "tan", "asin", "acos", "atan", "sinh", "cosh", "tanh", "floor", "ceil", "round", "isnan", "isfinite", "isinf", "erf", "erfc"]
            "jl2cpp::$name($(GenerateCppCode(args[1], info)))"
        # cast
        elseif IsType(Symbol(name))
            "static_cast<$(Symbol(name)|>eval|>ConvertType)>($(GenerateCppCode(args[1], info)))"
        # just call the function if it is known user-defined function
        elseif name in info.userDefFunc
            "$name($(join(map(x->"jl2cpp::eval_if_possible(" * GenerateCppCode(x, info) * ")", args), ", ")))"
        else
            error("unknown function $name")
        end
    end


    """
    Convert Julia code into C++ code.
    """
    function GenerateCppCode(ex::Expr, info::GenCppCodeInfo)::String
        # operator or function call
        if ex.head == :call
            # assign operators which equivalent form and function to operators of C++
            if ex.args[1] in [:+, :-, :*, :/, :%, :|, :&, :<<, :<, :>, :(<=), :(>=), :(==), :(!=)]
                if length(ex.args) < 3
                    "($(ex.args[1])$(GenerateCppCode(ex.args[2], info)))"
                else
                    ret = foldl((x,y)->begin
                        y_ = GenerateCppCode(y, info)
                        "$x $(ex.args[1]) $y_"
                    end, GenerateCppCode(ex.args[2], info), ex.args[3:end])
                    "($ret)"
                end
                # convert operators which only Julia has 
            elseif ex.args[1] in [:^, :$, :⊻, :.+, :.-, :.*, :./, :.^, :.<, :.<=, :.>, :.>=, :.==, :.!=, :>>, :>>>]
                arg1 = GenerateCppCode(ex.args[2], info)
                arg2 = GenerateCppCode(ex.args[3], info)
                if ex.args[1] == :^
                    "jl2cpp::pow($arg1, $arg2)" # TODO: implement matrix ^ scalar case (which is one of unique feature of julia)
                elseif ex.args[1] in [:&, :⊻] # old and new form of binary operator xor
                    "($arg1 ^ $arg2)"
                elseif ex.args[1] == :.+
                    "jl2cpp::add($arg1, $arg2)"
                elseif ex.args[1] == :.-
                    "jl2cpp::sub($arg1, $arg2)"
                elseif ex.args[1] == :.*
                    "jl2cpp::cwiseProduct($arg1, $arg2)"
                elseif ex.args[1] == :./
                    "jl2cpp::cwiseQuotient($arg1, $arg2)"
                elseif ex.args[1] == :.^
                    "jl2cpp::cwisePow($arg1, $arg2)"
                elseif ex.args[1] == :.<
                    "jl2cpp::cwiseLt($arg1, $arg2)"
                elseif ex.args[1] == :.<=
                    "jl2cpp::cwiseLte($arg1, $arg2)"
                elseif ex.args[1] == :.>
                    "jl2cpp::cwiseGt($arg1, $arg2)"
                elseif ex.args[1] == :.>=
                    "jl2cpp::cwiseGte($arg1, $arg2)"
                elseif ex.args[1] == :.==
                    "jl2cpp::cwiseEqual($arg1, $arg2)"
                elseif ex.args[1] == :.!=
                    "jl2cpp::cwiseNotEqual($arg1, $arg2)"
                elseif ex.args[1] == :>> # arith bitshift
                    "jl2cpp::arith_rshift($arg1, $arg2)" 
                elseif ex.args[1] == :>>> # logical bitshift
                    "jl2cpp::logic_rshift($arg1, $arg2)"
                end
            else
                # TODO: make unknown and not included function binded with the parent code
                ConvertFunction("$(ex.args[1])", length(ex.args) > 1? ex.args[2:end] : [], info)
            end

        # assign
        # TODO: consider distinguish lhv and rhv

        elseif ex.head ==:(=)
            decl = ""
            ret = []
            rhs = GenerateCppCode(ex.args[end], info)
            i = -1
            # tuple in lhs
            if isa(ex.args[1], Expr) && ex.args[1].head == :tuple
                # multiple assignment
                if ex.args[2] isa Expr && ex.args[2].head == :tuple
                    # ad-hoc for special case: swap
                    if length(ex.args[1].args) == 2 && symdiff(ex.args[1].args, ex.args[2].args) |> isempty
                        return "std::swap(" * join(map(ex.args[1].args) do x GenerateCppCode(x, info); end, ", ") * ")"
                    end
                    for i = 1:(ex.args[1].args|>length)
                        if ex.args[1].args[i] isa Expr && ex.args[1].args[i].head == :local
                            push!(ret, "auto $(GenerateCppCode(ex.args[1].args[i].args[1], info)) = $(GenerateCppCode(ex.args[2].args[i], info))")
                        else
                            push!(ret, "$(GenerateCppCode(ex.args[1].args[i], info)) = $(GenerateCppCode(ex.args[2].args[i], info))")
                        end
                    end
                else
                    varname = []
                    for i=1:length(ex.args[1].args)
                        if ex.args[1].args[i] isa Expr && ex.args[1].args[i].head == :local
                            decl *= "typename jl2cpp::ElType<decltype(std::get<$(i - 1)>($rhs))>::type $(ex.args[1].args[i].args[1]); "
                            push!(varname, ex.args[1].args[i].args[1])
                        else
                            push!(varname, GenerateCppCode(ex.args[1].args[i], info))
                        end
                    end
                    push!(ret, "std::tie($(join(varname, ", "))) = $rhs")
                end
            else
                if ex.args[2] isa Expr && ex.args[2].head in [:local, :(=), :+=, :-=, :*=, :/=, :%=, :|=, :&=, :<<=, :>>=, :^=, :$=, :⊻=, :.+=, :.-=, :.*=, :./=, :.^=]
                    vn = if ex.args[2].head == :local ex.args[2].args[1].args[1] else ex.args[2].args[1] end
                    push!(ret, "$(GenerateCppCode(ex.args[2], info))")
                    push!(ret, "$(GenerateCppCode(ex.args[1], info)) = $vn")
                #if ex.args[1] isa Expr && ex.args[1].head == :local
                    # normal vardecl w/ init
                #    push!(ret, "auto $(GenerateCppCode(ex.args[1].args[1], info)) = $(GenerateCppCode(ex.args[2], info))")
                else
                    push!(ret, "$(GenerateCppCode(ex.args[1], info)) = $(GenerateCppCode(ex.args[2], info))")
                end
            end
            return decl * join(ret, "; ")# * (length(ret) <= 1? ";": "")

        # assign operators which equivalent form and function to operators of C++
        elseif ex.head in [:+=, :-=, :*=, :/=, :%=, :|=, :&=, :<<=, :>>=]
            "$(GenerateCppCode(ex.args[1], info)) $(ex.head) $(GenerateCppCode(ex.args[2], info))"
        elseif ex.head in [:^=, :$=, :⊻=, :.+=, :.-=, :.*=, :./=, :.^=]
            arg1 = GenerateCppCode(ex.args[1], info)
            arg2 = GenerateCppCode(ex.args[2], info)
            if ex.args[1] == :^
                "$arg1 = jl2cpp::pow($arg1, $arg2)"
            elseif ex.args[1] in [:&, :⊻] # old and new form of binary operator xor
                "$arg1 = $arg1 ^ $arg2"
            elseif ex.args[1] == :.+=
                "$arg1 = $arg1 + $arg2"
            elseif ex.args[1] == :.-=
                "$arg1 = $arg1 - $arg2"
            elseif ex.args[1] == :.*=
                "$arg1 = cwiseProduct($arg1, $arg2)"
            elseif ex.args[1] == :./=
                "$arg1 = cwiseQuotient($arg1, $arg2)"
            elseif ex.args[1] == :.^=
                "$arg1 = cwisePow($arg1, $arg2)"
            end
        # chained comparison operators
        # FixMe: operand with side effects causes wrong result
        elseif ex.head == :comparison
            ret = []
            for i = 2:2:length(ex.args)
                push!(ret, GenerateCppCode(ex.args[i-1]) * " " * "$(ex.args[i])" * " " * GenerateCppCode(ex.args[i+1]))
            end
            join(ret, " && ")
        # block
        elseif ex.head == :block
            ret = ""
            lineStr = ""
            info.indent += 1
            indentStr = repeat("\t", info.indent)
            for i = 1:length(ex.args)
                if isa(ex.args[i], Expr) && ex.args[i].head == :line
                    lineStr = GenerateCppCode(ex.args[i], info)
                    continue
                end
                l = GenerateCppCode(ex.args[i], info)
                ret *= l[end] == '}'? "$indentStr$lineStr\n$indentStr$l\n": "$indentStr$l; $lineStr\n"
                lineStr = ""
            end
            info.indent -= 1
            indentStr = repeat("\t", info.indent)
            "{\n$ret$indentStr}"
        elseif ex.head == :let
            body = GenerateCppCode(ex.args[1], info)[3:end]
            vars = length(ex.args) > 1? foldl((x,y)->"$x$(GenerateCppCode(y, info));\n", "", ex.args[2:end]): ""
            "{\n$vars$body"
        elseif ex.head == :(::)
            "$(ConvertType(ex.args[2]|>eval)) $(ex.args[1])"
        # function
        elseif ex.head == :function
            if ex.args[1].head != :(::)
                error("function must be typed")
            end
            name = ex.args[1].args[1].args[1]
            args = ex.args[1].args[1].args[2:end]
            retty = ex.args[1].args[2] |> eval |>ConvertType
            # retty = (retty.name.name == :Array)? "jl2cpp::matrix": ConvertType(retty)

            push!(info.userDefFunc, String(name))
            argstrs = String[]
            initstrs = String[]
            templatestrs = String[]
            i = 0
            for a = args
                if typeof(a) != Expr || a.head != :(::)
                    error("function arguments must be typed")
                end
                cname = a.args[1]
                ty = eval(a.args[2])
                templatename = "T$i"
                if ty.name.name == :Array
                    cty = ty.parameters[1] |> ConvertType
                    if ty.parameters[2] == 1
                        push!(argstrs, "$templatename& $(cname)")
                        #push!(initstrs, repeat("\t", info.indent + 1) * "Map<Matrix<$cty, Dynamic, 1> > $cname(($cty *)$(cname)_jl2cpp_arg->ptr, $(cname)_jl2cpp_arg->rsize);")
                        # TODO : uncomment
                        push!(initstrs, repeat("\t", info.indent + 1) * "$cty *const __restrict $(cname)_jl2cpp_ptr = ($cty *)$(cname).data(); auto const $(cname)_jl2cpp_rsize = $(cname).size();")
                        push!(info.argArrays, cname)
                    elseif ty.parameters[2] == 2
                        push!(argstrs, "$templatename& $(cname)")
                        #push!(initstrs, repeat("\t", info.indent + 1) * "Map<Matrix<$cty, Dynamic, Dynamic> > $cname(($cty *)$(cname)_jl2cpp_arg->ptr, $(cname)_jl2cpp_arg->rsize, $(cname)_jl2cpp_arg->csize);")
                        # TODO: uncomment
                        push!(initstrs, repeat("\t", info.indent + 1) * "$cty *const __restrict $(cname)_jl2cpp_ptr = ($cty *)$(cname).data(); auto const $(cname)_jl2cpp_rsize = $(cname).rows(); auto const $(cname)_jl2cpp_csize = $(cname).cols();")
                        push!(info.argArrays, cname)
                    else
                        error("3+ dimension array is currently not supported.")
                    end
                    push!(templatestrs, "typename $templatename")
                    i += 1
                else
                    cty = ty |> ConvertType
                    push!(argstrs, "$cty $cname")
                end
            end
            argstr = join(argstrs, ", ")
            initstr = join(initstrs, "\n")
            body = "{\n" * initstr * GenerateCppCode(ex.args[2], info)[2:end]
            (templatestrs|>isempty? "": "template<$(join(templatestrs, ", "))> ") * "EIGEN_STRONG_INLINE $retty $name($argstr)\n$body\n$(GenerateWrapperFunction(ex, info))"
        # for
        elseif ex.head == :for
            function makeFor(loopvar, iteratee)
                # foreach-style for
                if isa(iteratee, Symbol)
                    "for(auto&& $loopvar : $iteratee)"
                elseif isa(iteratee, Expr)
                    # number-counting for
                    if iteratee.head == :(:)
                        if length(iteratee.args) == 2
                            b = GenerateCppCode(iteratee.args[1], info)
                            e = GenerateCppCode(iteratee.args[2], info)
                            "for(auto $loopvar = $b; $loopvar <= $e; ++$loopvar)"
                        elseif length(iteratee.args) == 3
                            b = GenerateCppCode(iteratee.args[1], info)
                            s = GenerateCppCode(iteratee.args[2], info)
                            e = GenerateCppCode(iteratee.args[3], info)
                            if iteratee.args[2] isa Integer
                                if iteratee.args[2] > 0
                                    "for(auto $loopvar = $b; $loopvar <= $e; $loopvar += $s)"
                                else
                                    "for(auto $loopvar = $b; $loopvar >= $e; $loopvar += $s)"
                                end
                            else
                                "for(auto $loopvar = $b; ($s > 0)? ($loopvar <= $e): ($loopvar >= $e); $loopvar += $s)"
                            end
                        else
                            error("counting for with illegal range operator")
                        end
                    else
                        "for(auto&& $loopvar : $(GenerateCppCode(iteratee, info)))\n$(GenerateCppCode(loopbody, info))"
                    end
                end
            end
            if ex.args[1].head == :(=) # non-nested loop
                header = makeFor(ex.args[1].args[1], ex.args[1].args[2])
                body = GenerateCppCode(ex.args[2], info)
                "$header$body"
            elseif ex.args[1].head ==:block # nested loop
                header = foldl((x,y)-> x * repeat("\t", info.indent) * y, map(x->makeFor(x.args[1], x.args[2]), ex.args[1].args[1:end]))
                body = GenerateCppCode(ex.args[2], info)
                "$header$body"
            end

        elseif ex.head == :while
            cond = GenerateCppCode(ex.args[1], info)
            body = GenerateCppCode(ex.args[2], info)
            "while($cond)$body"
        # if and ternary operator
        elseif ex.head == :if
            # syntax if
            if isa(ex.args[2], Expr) && ex.args[2].head == :block
                cond = GenerateCppCode(ex.args[1], info)
                tclause = GenerateCppCode(ex.args[2], info)
                if length(ex.args) > 2
                    fclause = GenerateCppCode(ex.args[3], info)
                    "if($cond)$(tclause)else $fclause"
                else
                    "if($cond)$(tclause)"
                end
            else
                # ternary operator
                if length(ex.args) != 3
                    error("ternary conditional operator without false clause is not supported")
                end
                cond = GenerateCppCode(ex.args[1], info)
                tclause = GenerateCppCode(ex.args[2], info)
                fclause = GenerateCppCode(ex.args[3], info)

                "(($cond)? ($tclause): ($fclause))"
            end
        
        # array reference
        elseif ex.head == :ref
            isIdxScalar(idx) = typeof(idx) != Expr || idx.head != :(:) # scalar index value
            isIdxRange(idx) = !isIdxScalar(idx) && length(idx.args) == 3 # [a:b]
            isIdxRangeHead(idx) = isIdxRange(idx) && idx.args[1] == 1
            isIdxRangeTail(idx) = isIdxRange(idx) && idx.args[2] == :end
            isIdxFullRange(idx) = !isIdxScalar(idx) && (length(idx.args) == 1 || length(idx.args) == 2 && idx.args[1] == 1 && idx.args[2] == :end) # [:], [1:end]
            isIdxStride(idx) = !isIdxScalar(idx) && length(idx.args) == 3

            arrayName = GenerateCppCode(ex.args[1], info)

            # reference indeces
            # [idx] or [idxb:idxe] or [rb:re, cb:ce] or [r, c]
            # defined not as constant values but as functions because intended to do lazy evaluation
            rb(e) = GenerateCppCode(e.args[2].args[1], info)
            re(e) = GenerateCppCode(e.args[2].args[2], info)
            cb(e) = GenerateCppCode(e.args[3].args[1], info)
            ce(e) = GenerateCppCode(e.args[3].args[2], info)
            r(e) = GenerateCppCode(e.args[2], info)
            c(e) = GenerateCppCode(e.args[3], info)
            idx(e) = GenerateCppCode(e.args[2], info)
            idxb(e) = GenerateCppCode(e.args[2].args[1], info)
            idxe(e) = GenerateCppCode(e.args[2].args[2], info)

            
            if length(ex.args) == 1
                "$arrayName(0)"
            elseif length(ex.args) == 2 # one-dimension access
                if isIdxScalar(ex.args[2])                              # [i]
                    if Symbol(arrayName) in info.argArrays
                        "$(arrayName)_jl2cpp_ptr[$(idx(ex)) - 1]"
                    else
                        "$(arrayName).data()[$(idx(ex)) - 1]"
                    end
                elseif isIdxFullRange(ex.args[2])                       # [:]
                    "$arrayName"
                elseif isIdxRangeHead(ex.args[2])                       # [1:i]
                    "$arrayName.head($(idx(ex)))"
                elseif isIdxRangeTail(ex.args[2])                       # [i:end]
                    "$arrayName.tail($arrayName.size() - $(idxb(ex)) + 1)"
                elseif isIdxRange(ex.args[2])                           # [i:j]
                    "$arrayName.segment($(idxb(ex)) - 1, $(idxe(ex)) - $(idxb(idx)) + 1)"
                elseif isIdxStride(ex.args[2])
                    error("stride access is currently not supported.")
                else
                    error("unknown reference to array")
                end
            elseif length(ex.args) == 3 # two-dimension access
                if isIdxStride(ex.args[2]) || isIdxStride(ex.args[3])
                    error("stride access is currently not supported.")
                elseif isIdxFullRange(ex.args[2])
                    if isIdxFullRange(ex.args[3])                       # [:, :]
                        "$arrayName"
                    elseif isIdxRangeHead(ex.args[3])                   # [:, 1:b]
                        "$arrayName.leftCols($(ce(ex)))"
                    elseif isIdxRangeTail(ex.args[3])                   # [:, a:end]
                        "$arrayName.rightCols($arrayName.cols() - $(cb(ex)) + 1)"
                    elseif isIdxRange(ex.args[3])                       # [:, a:b]
                        "$arrayName.middleCols($(eb(ex)) - 1, $(ce(ex)) - $(cb(ex)) + 1)"
                    elseif isIdxScalar(ex.args[3])                      # [:, c]
                        "$arrayName.col($(c(ex)) - 1)"
                    else
                        error("this line must be unreachable.")
                    end
                elseif isIdxRangeHead(ex.args[2])
                    if isIdxFullRange(ex.args[3])                       # [1:b, :]
                        "$arrayName.topRows($(re(ex)))"
                    elseif isIdxRangeHead(ex.args[3])                   # [1:b, 1:d]
                        "$arrayName.topLeftCorner($(re(ex)), $(ce(ex)))"
                    elseif isIdxRangeTail(ex.args[3])                   # [1:b, c:end]
                        "$arrayName.topRightCorner($(rb(ex)), $arrayName.cols() - $(cb(ex)) + 1)"
                    elseif isIdxRange(ex.args[3])                       # [1:b, c:d]
                        "$arrayName.block(0, $(cb(ex)) - 1, $(re(ex)), $(ce(ex)) - $(cb(ex)) + 1)"
                    elseif isIdxScalar(ex.args[3])                      # [1:b, c]
                        "$arrayName.col($(c(ex)) - 1).head($(re(ex)))"
                    else
                        error("this line must be unreachable.")
                    end
                elseif isIdxRangeTail(ex.args[2])
                    if isIdxFullRange(ex.args[3])                       # [a:end, :]
                        "$arrayName.bottomRows($arrayName.rows() - $(rb(ex)) + 1)"
                    elseif isIdxRangeHead(ex.args[3])                   # [a:end, 1:d]
                        "$arrayName.bottomLeftCorner($arrayName.rows() - $(rb(ex)) + 1, $(ce(ex)))"
                    elseif isIdxRangeTail(ex.args[3])                   # [a:end, c:end]
                        "$arrayName.bottomRightCorner($arrayName.rows() - $(rb(ex)) + 1, $arrayName.cols() - $(cb(ex)) + 1)"
                    elseif isIdxRange(ex.args[3])                       # [a:end, c:d]
                        "$arrayName.block($(rb(ex)) - 1, $(ex.args[3].args[1]|> GenerateCppCode) - 1, $arrayName.rows() - $(rb(ex)) + 1, $(ce(ex)) - $(cb(ex)) + 1)"
                    elseif isIdxScalar(ex.args[3])                      # [a:end, c]
                        "$arrayName.col($(c(ex)) - 1).tail($arrayName.rows() - $(rb(ex)) + 1))"
                    else
                        error("this line must be unreachable.")
                    end
                elseif isIdxRange(ex.args[2])
                    if isIdxFullRange(ex.args[3])                       # [a:b, :]
                        "$arrayName.middleRows($(rb(ex)) - 1, $(re(ex)) - $(rb(ex)) + 1)"
                    elseif isIdxRangeHead(ex.args[3])                   # [a:b, 1:d]
                        "$arrayName.block($(rb(ex)) - 1, 0, $(re(ex)) - $(rb(ex)) + 1, $(ce(ex)))"
                    elseif isIdxRangeTail(ex.args[3])                   # [a:b, c:end]
                        "$arrayName.block($(rb(ex)) - 1, $(cb(ex)) - 1, $(re(ex)) - $(rb(ex)) + 1, $arrayName.cols() - $(cb(ex)) + 1)"
                    elseif isIdxRange(ex.args[3])                       # [a:b, c:d]
                        "$arrayName.block($(rb(ex)) - 1, $(cb(ex)) - 1, $(re(ex)) - $(rb(ex)) + 1, $(ce(ex)) - $(cb(ex)) + 1)"
                    elseif isIdxScalar(ex.args[3])                      # [a:b, c]
                        "$arrayName.col($(c(ex)) - 1).segment($(rb(ex)) - 1, $(re(ex)) - $(rb(ex)) + 1)"
                    else
                        error("this line must be unreachable.")
                    end
                elseif isIdxScalar(ex.args[2])
                    if isIdxFullRange(ex.args[3])                       # [a, :]
                        "$arrayName.row($(r(ex)) - 1)"
                    elseif isIdxRangeHead(ex.args[3])                   # [a, 1:d]
                        "$arrayName.row($(r(ex)) - 1).head($(cb(ex)))"
                    elseif isIdxRangeTail(ex.args[3])                   # [a, c:end]
                        "$arrayName.row($(r(ex)) - 1).tail($arrayName.cols() - $(cb(ex)) + 1)"
                    elseif isIdxRange(ex.args[3])                       # [a, c:d]
                        "$arrayName.row($(r(ex)) - 1).segment($(cb(ex)) - 1, $(ce(ex)) - $(cb(ex)) + 1)"
                    elseif isIdxScalar(ex.args[3])                      # [a, c]
                        if Symbol(arrayName) in info.argArrays
                            "$(arrayName)_jl2cpp_ptr[($(r(ex)) - 1) + ($(c(ex)) - 1) * $(arrayName)_jl2cpp_rsize]"
                        else
                            "$(arrayName).data()[($(r(ex)) - 1) + ($(c(ex)) - 1) * $(arrayName)_jl2cpp_rsize]"
                        end
                    end
                else
                    error("this line must be unreachable.")
                end
            else
                error("3+ dimeinson array is currently not supported.")
            end
            #procArray(arrayName, reverse(ex.args[2:end]))
  
        elseif ex.head == :local
            ret = []
            function getIval(ex)
                if ex.args[2] isa Expr && ex.args[2].head in [:local, :(=), :+=, :-=, :*=, :/=, :%=, :|=, :&=, :<<=, :>>=, :^=, :$=, :⊻=, :.+=, :.-=, :.*=, :./=, :.^=]
                    vn = if ex.args[2].head == :local ex.args[2].args[1].args[1] else ex.args[2].args[1] end
                    push!(ret, "$(GenerateCppCode(ex.args[2], info)) ")
                    "$vn"
                else
                    GenerateCppCode(ex.args[2], info)
                end
            end
            for i in 1:length(ex.args)
                if isa(ex.args[i], Expr)
                    if ex.args[i].head == :(=)
                        if isa(ex.args[i].args[1], Expr) && ex.args[i].args[1].head == :(::)
                            tname = ex.args[i].args[1].args[2] |> eval |> ConvertType
                            vname = ex.args[i].args[1].args[1]
                            
                            ival = getIval(ex.args[i])
                            push!(ret, "$tname $vname = $ival")
                        else
                            vname = ex.args[i].args[1]
                            ival = getIval(ex.args[i])
                            push!(ret, "auto $vname = $ival")
                        end
                    elseif ex.args[i].head == :(::)
                        tname = ex.args[i].args[2] |> eval |> ConvertType
                        vname = ex.args[i].args[1]
                        push!(ret, "$tname $vname")
                    end
                else
                    # push!(ret, "auto $(ex.args[i])")
                    error("Local variable $(ex.args[i]) is declared without type annotation nor initialization.\n Please add at least one of them.")
                end
            end
            join(ret, "; ")
        elseif ex.head == :tuple
            # a,b,c=... form not in :block
            if isa(ex.args[end], Expr) && ex.args[end].head == :(=)
                decl = ""
                tie = "std::tie("
                rhs = GenerateCppCode(ex.args[end].args[end], info)
                for i = 1:(ex.args|>length) - 1
                    if isa(ex.args[i], Expr) && ex.args[i].head == :local
                        decl *= "decltype(std::get<$i>($rhs) $(GenerateCppCode(ex.args[i].args[1], info)); "
                        tie *= GenerateCppCode(ex.args[i].args[1], info)
                    else
                        tie *= GenerateCppCode(ex.args[i], info)
                    end
                    tie *= ", "
                end
                if isa(ex.args[end].args[end], Expr) && ex.args[end].args[1].head == :local
                    decl *= "decltype(std::get<$(ex.args|>length)>($rhs)) $(GenerateCppCode(ex.args[end].args[1].args[1], info)); "
                    tie *= GenerateCppCode(ex.args[end].args[1].args[1], info)
                else
                    tie *= GenerateCppCode(ex.args[end].args[1], info)
                end
                "$decl$tie) = $rhs"
            else
                # rhs tuple
                # "std::forward_as_tuple(" * join(map(x->GenerateCppCode(x, info), ex.args), ", ") * ")"
                "std::make_tuple(" * join(map(x->GenerateCppCode(x, info), ex.args), ", ") * ")"
            end
        # vector constructor
        # TODO: correct :vect, :hcat, :vcat
        elseif ex.head == :vcat
            if isa(ex.args[1], Expr) && ex.args[1].head == :row #matrix
                el = []
                for c in ex.args, r in c.args
                    push!(el, GenerateCppCode(r, info))
                end
                "(Matrix<jl2cpp::ElType<decltype(jl2cpp::get_one_elem($(el[1])))>::type, Dynamic, Dynamic>(jl2cpp::get_rows($(el[1])) * $(length(ex.args)),  jl2cpp::get_cols($(el[1])) * $(length(ex.args[1].args))) << $(join(el, ", "))).finished()"
            else # vertical vector
                el = []
                for c in ex.args
                    if c isa Expr && c.head == :(:) # TODO: implement range expression in vcat
                        error("range expression is currently not supported")
                    end
                    push!(el, GenerateCppCode(c, info))
                end
                "(Matrix<jl2cpp::ElType<decltype(jl2cpp::get_one_elem($(el[1])))>::type, Dynamic, Dynamic>(jl2cpp::get_rows($(el[1])) * $(length(ex.args)), jl2cpp::get_cols($(el[1]))) << $(join(el, ", "))).finished()"
            end
        elseif ex.head == :hcat # horizontal vector
            if ex.args|>isempty
                # empty array
                error("not implemented")
            else
                el = []
                for c in ex.args
                    push!(el, GenerateCppCode(c, info))
                end
                "(Matrix<jl2cpp::ElType<decltype(jl2cpp::get_one_elem($(el[1])))>::type, Dynamic, Dynamic>(jl2cpp::get_rows($(el[1])), jl2cpp::get_cols($(el[1])) * $(length(ex.args))) << $(join(el, ", "))).finished()"
            end
        elseif ex.head == :vect #1d array
            if ex.args|>isempty
                # empty array
                error("not implemented")
            else
                el = []
                for c in ex.args
                    push!(el, GenerateCppCode(c, info))
                end
                "(Matrix<jl2cpp::ElType<decltype($(el[1]))>::type, Dynamic, 1>($(length(ex.args))) << $(join(el, ", "))).finished()"
            end
        elseif ex.head == Symbol(".'")
            "$(GenerateCppCode(ex.args[1], info)).transpose()"
        elseif ex.head == Symbol("'")
            "$(GenerateCppCode(ex.args[1], info)).adjoint()"
        # macro
        elseif ex.head == :macrocall
            if ex.args[1] == Symbol("@fastmath")
                GenerateCppCode(ex.args[2], info)
            elseif ex.args[1] == Symbol("@printf")
                "printf($(join(map(x->GenerateCppCode(x, info), ex.args[2:end]), ", ")))"
            elseif ex.args[1] == Symbol("@test")
                "assert($(join(map(x->GenerateCppCode(x, info), ex.args[2:end]), " && ")))"
            elseif ex.args[1] == Symbol("@pragma")
                "#pragma $(ex.args[2])\n" * repeat("\t", info.indent) * GenerateCppCode(ex.args[3], info)
            elseif ex.args[1] == Symbol("@parallel")
                if !(ex.args[2] isa Expr && ex.args[2].head == :for)
                    error("not implemented")
                end
            elseif ex.args[1] == (Symbol("@threads"))
                "#pragma omp parallel for\n" * repeat("\t", info.indent) * GenerateCppCode(ex.args[2], info)
            elseif ex.args[1] isa Expr && ex.args[1].head == :(.) # module
                if ex.args[1].args[1] == :Threads && ex.args[1].args[2] == Expr(:quote, Symbol("@threads"))
                    "#pragma omp parallel for\n" * repeat("\t", info.indent) * GenerateCppCode(ex.args[2], info)
                else
                    "/*$(ex.args[1])*/ " * GenerateCppCode(ex.args[2], info)
                end
            else "/*$(ex.args[1])*/ " * GenerateCppCode(ex.args[2], info)
            end
        elseif ex.head == :using
        # line number for debug
        elseif ex.head == :line
            "/* line $(ex.args[1]) */"
        elseif ex.head == :return
            "return jl2cpp::eval_if_possible($(GenerateCppCode(ex.args[1], info)))"
        elseif ex.head == :continue
            "continue"
        elseif ex.head == :break
            "break"
        else
            "/** unknown expr: $(ex.head) **/"
        end
    end
    function GenerateCppCode(e::AbstractString, info::GenCppCodeInfo)::String
        return "\"" * e * "\""
    end
    function GenerateCppCode(e::Any, info::GenCppCodeInfo)::String
        return string(e)
    end

    # insert `return` to AST
    function InsertReturnSyntax(ex :: Expr)
        if ex.head == :function
            ex.args[2] = InsertReturnSyntax(ex.args[2])
            ex
        elseif ex.head == :block
            ex.args[end] = InsertReturnSyntax(ex.args[end])
            ex
        elseif ex.head == :let
            InsertReturnSyntax(ex.args[1])
            ex
        elseif ex.head == :if
            # syntax if
            if isa(ex.args[2], Expr)
                ex.args[2] = InsertReturnSyntax(ex.args[2])
                ex.args[3] = InsertReturnSyntax(ex.args[3])
            else
                # ternary operator
                ex = Expr(:return, ex)
            end
            ex
        elseif ex.head in [:while, :for, :return]
            ex
        else
            # if the cluause that should be returned is a assignment, discard lvalue and modify it to return syntax. 
            # this may cause compatibility issue in (very rare) case that the lvalue is a global variable or causes side effect by the assignment.
            if isa(ex, Expr) && ex.head == :(=)
                Expr(:return, ex.args[2])
            else
                Expr(:return, ex)
            end
        end
    end

    function InsertReturnSyntax(ex)
        Expr(:return, ex)
    end

    function IsVarExist(env :: Vector{Dict{Symbol, DataType}}, var :: Symbol) :: Bool
        if env|>isempty
            return false
        end
        for i=length(env):-1:1
            if haskey(env[i], var)
                return true
            end
        end
        false
    end

    """
    Insert explicit `local` node to the first assignment of local variables
    in order to make variable decleration easy to be converted.
    """
    function InsertVarDecl(ex::Expr, env :: Vector{Dict{Symbol, DataType}})
        function NewScope!(env :: Vector{Dict{Symbol, DataType}})
            push!(env, Dict{Symbol, DataType}())
        end
        function DeleteScope!(env :: Vector{Dict{Symbol, DataType}})
            pop!(env)
        end
        function RegisterVar!(env :: Vector{Dict{Symbol, DataType}}, var::Any, typ::DataType)
            if isempty(env)
                NewScope!(env)
            end
            if typeof(var) != Symbol
                println(var)
                join(stacktrace(),"\n") |> println
                error("not symbol")
            end
            env[end][var] = typ
        end

        if ex.head == :function
            if isa(ex.args[1], Expr) && ex.args[1].head == :(::)
                NewScope!(env)
                for x in ex.args[1].args[1].args
                    if isa(x, Expr) && x.head == :(::)
                        RegisterVar!(env, x.args[1], x.args[2]|>eval)
                    else
                        RegisterVar!(env, x, Any)
                    end
                end
                InsertVarDecl(ex.args[2], env)
                DeleteScope!(env)
            else
                NewScope!(env)
                if length(ex.args[1].args) >= 2
                    # println(ex.args[1].args)
                    foldl((x,y)->RegisterVar!(env, y, Any), ex.args[1].args[1:end])
                end
                InsertVarDecl(ex.args[2], env)
                DeleteScope!(env)
            end
        
        elseif ex.head == :for
            NewScope!(env)
            if ex.args[1].head == :block
                foldl((x,y)->RegisterVar!(env, y.args[1], Any), ex.args[1].args[1:end])
            else
                RegisterVar!(env, ex.args[1].args[1], Any)
            end
            InsertVarDecl(ex.args[2], env)
            DeleteScope!(env)
        elseif ex.head == :while
            NewScope!(env)
            if ex.args[1].head == :block
                foldl((x,y)->RegisterVar!(env, y.args[1], Any), ex.args[1].args[1:end])
            else
                RegisterVar!(env, ex.args[1].args[1], Any)
            end
            InsertVarDecl(ex.args[2], env)
            DeleteScope!(env)
        elseif ex.head == :block
            for i in 1:length(ex.args)
                if ex.args[i].head == :line
                    continue
                else
                    ex.args[i] = InsertVarDecl(ex.args[i], env)
                end
            end
        elseif ex.head == :(=)
            ex.args[2] = InsertVarDecl(ex.args[2], env)
            if isa(ex.args[1], Expr) && ex.args[1].head == :(::) && isa(ex.args[1].args[1], Symbol)
                if !IsVarExist(env, ex.args[1].args[1])
                    RegisterVar!(env, ex.args[1].args[1], ex.args[1].args[2] |> eval)
                    ex = Expr(:local, ex)
                end
            elseif isa(ex.args[1], Expr) && ex.args[1].head == :tuple
                for i in 1:(ex.args[1].args|>length)
                    if !IsVarExist(env, ex.args[1].args[i])
                        RegisterVar!(env, ex.args[1].args[i], Any)
                        ex.args[1].args[i] = Expr(:local, ex.args[1].args[i])
                    end
                end
            else
                if isa(ex.args[1], Symbol) && !IsVarExist(env, ex.args[1])
                    RegisterVar!(env, ex.args[1], Any) # TODO : type inference
                    ex = Expr(:local, ex)
                end
            end
        elseif ex.head == :tuple && isa(ex.args[end], Expr) && ex.args[end].head == :(=)
            for i = 1 : ex.args |> length - 1
                if !IsVarExist(env, ex.args[i])
                    RegisterVar!(env, ex.args[i], Any)
                    ex = Expr(:local, ex.args[i])
                end
            end
            if !IsVarExist(env, ex.args[end].args[1])
                RegisterVar!(env, ex.args[end].args[1], Any)
                ex.args[end].args[1] = Expr(:local, ex.args[end].args[1])
            end
        elseif ex.head == :macrocall
            for i = 2:length(ex.args)
                InsertVarDecl(ex.args[i], env)
            end
        elseif ex.head == :local
            for i = 1:length(ex.args)
                if ex.args[i] isa Symbol
                    RegisterVar!(env, ex.args[i], Any)
                    continue
                elseif ex.args[i] isa Expr
                    e = ex.args[i]
                    if e.head == :(::)
                        RegisterVar!(env, e.args[1], e.args[2]|>eval)
                        continue
                    elseif e.head == :(=)
                        InsertVarDecl(e.args[2], env)
                        lhs = e.args[1]
                        if lhs isa Expr && lhs.head == :(::)
                            RegisterVar!(env, lhs.args[1], lhs.args[2]|>eval)
                            continue
                        elseif lhs isa Symbol
                            RegisterVar!(env, lhs, Any)
                            continue
                        else
                            error("this cannot be reached")
                        end
                    else
                        error("wrong syntax")
                    end
                else
                    error("wrong syntax")
                end
            end
        end
        ex
    end
    InsertVarDecl(ex :: Expr) = InsertVarDecl(ex, Vector{Dict{Symbol, DataType}}())
    InsertVarDecl(ex :: Any, env :: Vector{Dict{Symbol, DataType}}) = ex

    # make form of `f(x)=x*2` function to `function f(x); x*2; end`
    function makeShortSyntaxFunctionLong(ex :: Expr)
        if ex.head == :(=) && ex.args[1] isa Expr &&
            (ex.args[1].head == :call ||
            (ex.args[1].head == :(::) && ex.args[1].args[1] isa Expr && ex.args[1].args[1].head == :call)
            )
                ex.head = :function
        end
        map!(makeShortSyntaxFunctionLong, ex.args, ex.args)
        ex
    end
    makeShortSyntaxFunctionLong(x) = x

    # replace "!" in function name to "_"
    function sanitizeFuncName(ex::Expr)
        if ex.head == :call && ex.args[1] isa Symbol
            ex.args[1] = Symbol(replace(string(ex.args[1]), "!", "_"))
            if ex.args|>length > 2
                map!(sanitizeFuncName, ex.args[2:end], ex.args[2:end])
            end
        else
            map!(sanitizeFuncName, ex.args, ex.args)
        end
        ex
    end
    sanitizeFuncName(x) = x

    function WriteCppFile(src::String, filename::String)
        open(filename * ".cpp", "a") do os
            println(os, src)
        end
    end
    function CompileCpp(filename :: String)
        if haskey(ENV, "NO_COMPILE") && ENV["NO_COMPILE"] == "1"
            info("no compiling")
            return
        end
        ext = Libdl.dlext
        run(`$CC $C_OPT $filename.cpp -o $filename.$ext`)
    end

    #TODO: function translation
    #TODO: type inference
    function ConvertFunctionDefinition(ex :: Expr, filename::String, info::GenCppCodeInfo)
        #ex|>Meta.show_sexpr
        if ex.head != :function
            error("this cannot be reached")
        end
        if ex.args[1].head != :(::)
            error("functions must be typed")
        end

        origname = ex.args[1].args[1].args[1]
        funname = replace(string(ex.args[1].args[1].args[1]), "!", "_")
        funname = "$(funname)_jl2cpp_wrapper"
        retty = eval(ex.args[1].args[2])

        if retty.name.name != :Void
            ex = ex |> InsertReturnSyntax
        end

        code = GenerateCppCode(ex |> sanitizeFuncName |> InsertVarDecl, info)
        WriteCppFile(code, filename)

        callexpr = Expr(:call, :ccall,
            Expr(:tuple, Expr(:quote, Symbol(funname)), joinpath(pwd(), "$filename." * Libdl.dlext)),
            retty.name.name == :Array? Expr(:curly, :(Jl2Cpp.CMatrix), retty.parameters[1]): retty.name.name == :Tuple? :(Jl2Cpp.CTuple): retty,
            Expr(:tuple)
        )
        freeexpr = Expr(:call, :ccall,
        Expr(:tuple, Expr(:quote, Symbol(funname)), joinpath(pwd(), "$filename." * Libdl.dlext)),
        Void,
        Expr(:tuple)
    )
        # repair original function name because might be modified
        ex.args[1].args[1].args[1] = origname
        precall_code = Expr[]
        push!(precall_code, :(gc_enable(false)))
        for arg in ex.args[1].args[1].args[2:end]
            if arg.head != :(::)
                error("all arguments must be typed")
            end

            ty = eval(arg.args[2])
            if ty.name.name == :Array
                conv_code = Expr(:(=))
                if ty.parameters[2] == 1
                    conv_code = :(tmp = Ref(Jl2Cpp.CMatrix{$(ty.parameters[1])}(pointer($(arg.args[1])), length($(arg.args[1])), 1)))
                    push!(callexpr.args[4].args, :(Ref{Jl2Cpp.CMatrix{$(ty.parameters[1])}}))
                elseif ty.parameters[2] == 2
                    conv_code = :(tmp = Ref(Jl2Cpp.CMatrix{$(ty.parameters[1])}(pointer($(arg.args[1])), size($(arg.args[1]), 1), size($(arg.args[1]), 2))))
                    push!(callexpr.args[4].args, :(Ref{Jl2Cpp.CMatrix{$(ty.parameters[1])}}))
                else
                    error("3+ dimension array is currently not supported")
                end
                conv_code.args[1] = Symbol("$(arg.args[1])_jl2cpp_arg")
                push!(precall_code, conv_code)
                push!(callexpr.args, Symbol("$(arg.args[1])_jl2cpp_arg"))

                # push!(precall_code, :(println($(Symbol("$(arg.args[1]i)_jl2cpp_arg")))))
            else
                push!(callexpr.args[4].args, arg.args[2])
                push!(callexpr.args, arg.args[1])
            end
        end

        postcall_code = []
        if retty.name.name == :Array
            push!(postcall_code, :(jl2cpp_ret = Jl2Cpp.ConvertToArray(jl2cpp_ret)))
        elseif retty.name.name == :Tuple
            push!(postcall_code, :(jl2cpp_ret = Jl2Cpp.ConvertToTuple($retty, jl2cpp_ret)))
        end
        #push!(postcall_code, :(gc_enable(true)))

        ex.args[2] = Expr(:block)
        append!(ex.args[2].args, precall_code)
        if retty.name.name == :Void
            push!(ex.args[2].args, callexpr)
            append!(ex.args[2].args, postcall_code)
        else
            push!(ex.args[2].args, Expr(:(=), Symbol("jl2cpp_ret"), callexpr))
            append!(ex.args[2].args, postcall_code)
            push!(ex.args[2].args, Expr(:return, Symbol("jl2cpp_ret")))
        end
        ex
    end

    function CppMacroImpl(ex::Expr, filename::String, info::GenCppCodeInfo)
        if ex.head == :block
            for i in 1:length(ex.args)
                CppMacroImpl(ex.args[i], filename, info)
            end
        elseif ex.head == :function
            ex = ConvertFunctionDefinition(ex, filename, info)
        elseif ex.head == :macrocall
            if ex.args[1] == Symbol("@inline") # ignore @inline because inline prefix is always added in C++ code
                CppMacroImpl(ex.args[2], filename, info)
            end
        elseif ex.head == :(=)
            error("not implemented")
        # elseif ex.head == :line
        else
            return ex
        end
        return ex
    end


    macro cpp(ex::Expr)
        putInfo("CC: $CC")
        putInfo("C_OPT: $C_OPT")

        filename = "jl2cpp_"*(PROGRAM_FILE|>basename|>splitext)[1]
        
        if isfile(filename * ".cpp")
            rm(filename * ".cpp")
        end
        if haskey(ENV, "JL2CPP_DEBUG") || "-D" in ARGS
            println("Debug mode")
        else
            push!(CPP_PREDEFS, "#define EIGEN_NO_DEBUG 1")
        end
        # cppfree = DLL_EXPORT * "void free_ptr(void* p){free(p);}\n"
        #WriteCppFile(join(CPP_PREDEFS, "\n") * "\n" * join(CPP_INCLUDES, "\n") * "\n" * join(CPP_USINGS, "\n") * "\n" * cppfree, filename)
        WriteCppFile(join(CPP_PREDEFS, "\n") * "\n" * join(CPP_INCLUDES, "\n") * "\n" * join(CPP_USINGS, "\n") , filename)

        ex = ex|>makeShortSyntaxFunctionLong
        for f in RECIPES
            f(ex)
        end
        info = GenCppCodeInfo(0, Set([]), Set([]))
        ret = CppMacroImpl(ex, filename, info)
        CompileCpp(filename)
        esc(ret)
    end
    export @cpp

    """
    Make definitions of both original version of passed function and compiled version of that.
    """
    macro test_func(ex :: Expr)
        ret = Expr(:block, copy(ex))
        name=ex.args[1].args[1].args[1]
        ex.args[1].args[1].args[1] = Symbol(name, "_cpp")
        # push!(ret.args, :(println("Start compiling ", $name)))
        # push!(ret.args, :(flush(STDOUT)))
        # push!(ret.args, :(tic()))
        push!(ret.args, :(@Jl2Cpp.cpp $ex))
        # push!(ret.args, :(println("Done. time: $(toq())s")))
        # push!(ret.args, :(flush(STDOUT)))
        esc(ret)
    end
    export test_func

    macro test_run(times :: Int64, fun :: Expr, cpp_only=false)
        prepare_data = Expr(:block)
        orig_expr = Expr(:block)
        cpp_expr = Expr(:block)
        call_orig = Expr(:(=), :orig_ret, Expr(:call, fun.args[1]))
        call_cpp = Expr(:(=), :cpp_ret, Expr(:call, Symbol(fun.args[1], "_cpp")))
        valid_args = Expr(:block)
        i = 0
        for a in fun.args[2:end]
            push!(prepare_data.args, Expr(:(=), Symbol("arg", i), a))
            push!(orig_expr.args, Expr(:(=), Symbol("orig_arg", i), Expr(:call, :copy,Symbol("arg", i))))
            push!(cpp_expr.args, Expr(:(=), Symbol("cpp_arg", i), Expr(:call, :copy,Symbol("arg", i))))
            push!(call_orig.args[2].args, Symbol("orig_arg", i))
            push!(call_cpp.args[2].args, Symbol("cpp_arg", i))
            push!(valid_args.args,
                Expr(:if, Expr(:call, :!, Expr(:call, :isapprox, Symbol("cpp_arg", i), Symbol("orig_arg", i))),
                    Expr(:call, :warn, "argument #", i+1, " is not expected value.")
                )
            )
            i += 1
        end
        push!(orig_expr.args, call_orig)
        push!(cpp_expr.args, call_cpp)
        quote
            local orig_time = 0.0
            local cpp_time = 0.0
            for i=1:$times
                info("Start iteration #$i")
                flush(STDOUT)
                $prepare_data
                if !$cpp_only
                    info("Runninng original")
                    flush(STDOUT)
                    tic()
                    $orig_expr
                    orig_time += toq()
                end
                info("Runninng cpp ver")
                flush(STDOUT)
                tic()
                $cpp_expr
                cpp_time += toq()
                if !$cpp_only
                    $valid_args
                    if cpp_ret != orig_ret && !(cpp_ret ≈ orig_ret)
                        warn("return value is wrong.")
                    end
                end
            end
            if !$cpp_only
                orig_time /= $times
                cpp_time /= $times
                println("orig: $(orig_time)s, cpp: $(cpp_time)s")
                println("cpp/orig ratio = ", cpp_time/orig_time)
                (orig_time, cpp_time)
            else
                cpp_time /= $times
                println("cpp: $(cpp_time)s")
                (0.0, cpp_time)
            end
        end|>esc
    end
    export test_run
end

#=
TODO:

=#

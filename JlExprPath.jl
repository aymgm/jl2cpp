module JlExprPath
# preparing a special type represents Expr because leaf nodes with non-boxing value can't be referenced
ExprRef = Tuple{Expr, Int64} # (expr, idx) if idx == 0 represents expr itself, otherwise expr.args[idx]
getValue(x::ExprRef) = x[2] == 0? x[1]: x[1].args[x[2]]
setValue!(x::ExprRef, v) = if x[2] == 0; x = (v, 0) else; x[1].args[x[2]] = v; end
#Base.show(io, x::Tuple{Expr, Int64}) = print(io, get(x))

ExprVec = Vector{ExprRef}
getValue(x::ExprVec) = x[end][2] == 0? x[end][1]: x[end][1].args[x[end][2]]
setValue!(x::ExprVec, v) = if x[end][2] == 0; x[end] = (v, 0) else; x[end][1].args[x[end][2]] = v; end

ExprVecVec = Vector{ExprVec}
getValue(x::ExprVecVec) = map(getValue, x)
# make(x::Vector{Expr})
# Base.show(io, x::Vector{Tuple{Expr, Int64}}) = print(io, get(x))
Pattern = Tuple{AbstractString, AbstractString, AbstractString}
PatternVec = Vector{Pattern}
export getValue, setValue!

function evalVerbFunc(name::AbstractString, nodes::ExprVecVec, verb::AbstractString)::String
	if name == ""
		if isempty(nodes)
			return "\"\""
		else
			if nodes|>length != 1
				warn("Got 2 or more nodes from query '$verb'. the query at here should return only one node.")
			end
			v = nodes[1]|>getValue
			return  v isa Expr? "\"$(v.head)\"": "\"$v\""
		end
	elseif name == "length"
		return string(length(nodes))
	elseif name == "pos"
		if isempty(nodes)
			error("function '$name' requires just one node but query '$verb' returned no nodes")
		else
			if nodes|>length != 1
				warn("Got 2 or more nodes from query '$verb' in function '$name'. the query at here should return only one node.")
			end
			v = nodes[1]
			if length(v) < 2 # root node
				return "0"
			end
			return string(v[end][2] == 0? v[end - 1][2]: v[end][2])
		end
	end
end

function evalVerb(verb::AbstractString, curnode::ExprVec)::Bool
	if verb == nothing || verb == ""
		return true
	end
	verb = replace(verb, r"@(\d+)", s"@(*[\1])")
	r = r"@(?<name>[a-z]*)
		\(
			(?<body>(?:
				(?P<v>
					\(((?>[^\(\)]+)|(?P>v))*\)
				)
				|[^\(]
			)+?)
		\)"x
	verb = replace(verb, r, s->begin
		m = match(r, s)
		return evalVerbFunc(m[:name], findExpr(m[:body], curnode), verb)
	end)

	local ret
	try
		ret = verb|>parse|>eval
	catch
		error("$verb is illegal expression")
	end
	if ret isa Bool
		return ret
	else
		error("$verb is not boolean expression")
	end
end

function testNode(ptn::Tuple{AbstractString, AbstractString, AbstractString}, er::ExprVec)::Bool
	if ptn |> isempty; return false; end
	axis, name, verb = ptn
	m = match(r"^\s*(\d+)\s*$", verb)
	if m != nothing
		nodenum = (length(er) > 1 && er[end][2] == 0? er[end - 1][2]: er[end][2])
		if parse(Int64, m.captures[1]) != nodenum
			return false
		end
		verb = ""
	end
	m = match(r"^\s*end\s*$", verb)
	if m != nothing
		nodenum = (length(er) > 1 && er[end][2] == 0? er[end - 1][2]: er[end][2])
		nodelen = (length(er) > 1 && er[end][2] == 0? er[end - 1][1].args|>length: er[end][1].args|>length)
		if nodenum != nodelen
			return false
		end
		verb = ""
	end
	if name == "*"
		return evalVerb(verb, er)
	end

    e = getValue(er)
    if e isa Expr
        if "$(e.head)" != name; return false; end
    else
        if "$e" != name; return false; end
	end
	return evalVerb(verb, er)
end

function childAxis(ptn::Pattern, src::ExprVec)::ExprVecVec
	ret = []
	s = getValue(src)
	if !(s isa Expr)
		return []
	end
	for i in 1:length(s.args)
		cv = [src..., (s, i)]
		if testNode(ptn, cv)
			push!(ret, cv)
		end
	end
	ret
end

function ancestorAxis(ptn::Pattern, src::ExprVec)::ExprVecVec
	if src|>length < 2
		throw("no ancestors")
		return []
	end
	ret::ExprVecVec = []
	for i = 1:(src|>length - 1)
		if testNode(ptn, src[1:i])
			push!(ret,src[1:i])
		end
	end
	ret
end

function ancestorOrSelfAxis(ptn::Pattern, src::ExprVec)::ExprVecVec
	if src|>length < 1
		throw("source is empty")
		return []
	end
	ret::ExprVecVec = []
	for i = 1:length(src)
		if testNode(ptn, src[1:i])
			push!(ret,src[1:i])
		end
	end
	ret
end

# filter src and its all descendants by ptn and return vector of them
function descendantOrSelfAxis(ptn::Pattern, src::ExprVec)::ExprVecVec
	ret = []
	s = getValue(src)
	if testNode(ptn, src)
		push!(ret, src)
	end
	if !(s isa Expr)
		return ret
	end
	for i in 1:length(s.args)
		append!(ret, descendantOrSelfAxis(ptn, [src..., (s, i)]))
	end
	ret
end

function descendantAxis(ptn::Pattern, src::ExprVec)::ExprVecVec
	if !(s isa Expr)
		return []
	end
	ret = []
	s = getValue(src)
	for i in 1:length(s.args)
		append!(ret, descendantOrSelfAxis(ptn, [src..., (s, i)]))
	end
	ret
end

function findExprImp(ptn::Pattern, srcs::ExprVecVec)::ExprVecVec
    if ptn|>isempty; return srcs; end
    axis, name, verb = ptn
	ret = []
	for src in srcs
		s = getValue(src)
		if axis == "self"
			if testNode(ptn, src)
				push!(ret, src)
			end
		elseif axis == "parent"
			if src|>length > 1 && testNode(ptn, src[1:(end - 1)])
				push!(ret, src[1:(end - 1)])
			end
		elseif axis == "child"
			append!(ret, childAxis(ptn, src))
		elseif axis == "ancestor"
			append!(ret, ancestorAxis(ptn, src))
		elseif axis == "ancestor-or-self"
			append!(ret, ancestorOrSelfAxis(ptn, src))
		elseif axis == "descendant"
			append!(ret, descendantAxis(ptn, src))
		elseif axis == "descendant-or-self"
			append!(ret, descendantOrSelfAxis(ptn, src))
		elseif axis == "root"
			ret = testNode(ptn, src[1:1])? [src[1:1]]: []
			break
		elseif axis == "func"
			error("func axis is available only in verb clause(i.e. in [...])")
		else
			throw("unknown axis $axis")
		end
	end
    ret
end

function parsePathString(str::AbstractString)::PatternVec
	ret = []
	if isempty(str)
		throw("invalid path")
	end

	# split path by `/` considering the `/` is out of `[...]`
	for (i, m) in enumerate(eachmatch(r"(?<=^|/)(?:(?P<v>\[((?>[^\[\]]+)|(?P>v))*\])|[^\[/])*", str))
		s = m.match
		if s == ""
			push!(ret, ((i == 1? "root": "descendant-or-self"), "*", ""))
			continue
		end
		rm = match(r"^(?:([-a-z]+?)\s*::\s*)?([^\s\[]+?)\s*(?:\[(.+)\])?\s*$", s)
		if rm isa Void
			throw("syntax error in $(s)")
		end
		axis, name, verb = rm.captures[1:3]
		if name == "."
			axis = "self"
			name = "*"
		end
		if name == ".."
			axis = "parent"
			name = "*"
		end
		axis = axis isa Void? "child": axis
		verb = verb isa Void? "": verb
		push!(ret, (axis, name, verb))
	end
	ret
end

function findExpr(ptn::AbstractString, src::ExprVecVec)::ExprVecVec
	v = copy(src)
	for p in parsePathString(ptn)
		v = findExprImp(p, v)
		if isempty(v)
			return []
		end
	end
	v|>unique
end
findExpr(ptn::AbstractString, src::ExprVec)::ExprVecVec = findExpr(ptn, [src])
findExpr(ptn::AbstractString, src::Expr)::ExprVecVec = findExpr(ptn, [[(src, 0)]])
export findExpr
end
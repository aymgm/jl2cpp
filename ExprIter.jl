module JlTuner
    type ExprState
        stack::Array{Tuple{Expr, Int64}} 
    end
    Base.copy(e::ExprState) = ExprState([i for i in e.stack])
    Base.start(e::Expr)::ExprState = ExprState([(e, 0)])
    Base.done(it::Expr, state::ExprState)::Bool = state.stack|>length == 0 || (state.stack|>length == 1 && state.stack[end][2] > state.stack[end][1].args|>length)
    function Base.next(it::Expr, state::ExprState)::Tuple{Any, ExprState}
        function promoteStack!(state::ExprState)
            if state.stack |> length > 0
                state.stack[end] = (state.stack[end][1], state.stack[end][2] + 1)
            end
        end
        function windStack!(state::ExprState)
            while state.stack |> length > 0 && state.stack[end][1].args |> length <= state.stack[end][2]
                pop!(state.stack)
            end
            promoteStack!(state)
        end

        s = copy(state)
        r = ()
        while true
            if s.stack |> isempty
                throw("expr stack must not be empty")
            end
            (e, i) = s.stack[end]

            if e |> typeof != Expr
                throw("non-expr in expr stack")
            end

            if i == 0
                promoteStack!(s)
                return (e, s)
            else
                if e.args[i] |> typeof == Expr
                    push!(s.stack, (e.args[i], 0))
                    continue
                else
                    windStack!(s)
                    return (e.args[i], s)
                end
            end
        end
    end
#=
:(  )
=#
    function interpretMetaInstruction(src::Any, ptn::Expr, env::Dict{Symbol, Expr})::Bool

    end
    function match(src::Expr, ptn::Expr, env::Dict{Symbol, Expr})::Bool
        sitr = start(src)
        pitr = start(ptn)
        while true
            if done(ptn, pitr)
                return true
            end
            if done(src, sitr)
                return false
            end
            (p, pitr) = next(ptn, pitr)
            (s, sitr) = next(src, sitr)

            if p |>typeof != Expr
                if p != s
                    return false
                end
                continue
            end
            if p.head == :macrocall
                # meta instructions
                #if 
            else
                if p.head != s.head
                    return false
                end
                continue
            end
        end
        return true
    end
end
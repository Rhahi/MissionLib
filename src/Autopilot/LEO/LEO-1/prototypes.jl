# prototype functions, later to be implemented into library

function query!(ch::Channel{Tuple{Symbol, Condition}}, command::Symbol)
    response = Condition()
    put!(ch, (command, response))
    answer = wait(response)
    return answer
end

"Solution to direction"
function s2d(sp::Spacecraft, sol)

end

# This file contains code for interfacing with Airsim

function targets_from_file(filename::String)::Array{Target,2}

    lines = readlines(filename)
    num_lines = length(lines)

    # Store data in lists
    positions = []
    headings = []
    unique_names = Set([])
    for l in lines
        splits = split(l, " ")
        name = strip(splits[2], ['[', ']'])
        # List of chars to exclude
        fchars = ['X', 'Y', 'Z', '=', 'P', 'Y', 'R']
        if splits[3][1] == 'X'
            push!(positions, parse(Int, map(x -> strip(x,fchars), splits[3:5])))
        else
            push!(headings, parse(Int, map(x -> strip(x, fchars), splits[3:5])))
        end
        push!(unique_names, name)
    end
    num_targets = length(unique_names)
    horizon = div(num_lines, (2 * num_targets))
    println(positions)
    println(headings)
    targets = Array{Target,2}(undef, horizon, num_targets)

end


@testset "test_file_parse" begin
    targets_from_file("./src/mdma_greedy/trajectory1.txt")
end

# This file contains code for interfacing with Airsim

function targets_from_file(filename::String)::Array{Target,2}

    lines = readlines(filename)
    num_lines = length(lines)

    # Store data in dictionaries

    ppdata = []
    hdata = []
    current_name = ""
    positions = []
    headings = []
    # Target names
    unique_names = []
    for l in lines
        splits = split(l, " ")
        name = strip(splits[2], ['[', ']'])
        if name != current_name
            println(name, current_name)
            current_name = name
            push!(ppdata, positions)
            push!(hdata, headings)
            positions = []
            headings = []
        end
        # List of chars to exclude
        fchars = ['X', 'Y', 'Z', '=', 'P', 'Y', 'R']

        if splits[3][1] == 'X'
            push!(positions, map(y -> parse(Float64, strip(y, fchars)), splits[3:5]))
        else
            push!(headings, map(y -> parse(Float64, strip(y, fchars)), splits[3:5]))
        end
        if !(name in unique_names)
            push!(unique_names, name)
        end
    end
    num_targets = length(unique_names)
    horizon = div(num_lines, (2 * num_targets))
    println(horizon)
    println(num_targets)
    println(ppdata)
    println(hdata)
    println(unique_names)

    # trajectory = Array{Target,2}(undef, horizon, num_targets)
    # for t in 1:horizon
    #     targets = Array{Target,1}(undef, num_targets)
    #     for i in 1:num_targets
    #         j = t + (num_targets * i - 1)
    #         println(t, i, j, positions[j])
    #         # targets[i] = Target(
    #     end
    # end


end


@testset "test_file_parse" begin
    # targets_from_file("./src/mdma_greedy/trajectory1.txt")
end

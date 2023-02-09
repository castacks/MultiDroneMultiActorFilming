# This file contains code for interfacing with Airsim

import JSON

function targets_from_json(filename::string)::Array{Target, 2}

   f = open(filename)
   json_root = JSON.parse(f.read())
   f.close()

   println!(json_root)

end


@testset "test_file_parse" begin
    # targets_from_file("./src/mdma_greedy/trajectory1.txt")
end

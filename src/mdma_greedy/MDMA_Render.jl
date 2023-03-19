using Cairo
export RenderConf, draw_target, draw_targets, draw_frames, draw_path

struct RenderConf
    ppm::Float64 # Pixels per meter for grid size
    buf::Int64   # Buffer grid surrounding the image
    draw_paths::Bool # Render paths
    draw_old_states::Bool # Set to false if you want only current state shown
end


function draw_background(cr::CairoContext, width, height)
    save(cr);
    set_source_rgb(cr,1,1,1);    # white
    rectangle(cr,0.0,0.0,width,height); # background
    fill(cr);
    restore(cr);
end

function draw_grid(g::MDMA_Grid, cr::CairoContext, point_size, ppm, buf)
    set_source_rgba(cr, 0, 0, 0, 0.3);
    x,y,z = dims(g)
    for i in 1:x
        for j in 1:y
            arc(cr, i*ppm + buf*ppm, j*ppm+ buf*ppm, point_size, 0, 2*pi);
            fill(cr);
        end
    end
end

# function draw_target(cr::CairoContext, t::Target, ppm, size, buf)
#     save(cr)
#     set_source_rgba(cr, 0, 0.3, 0.5, 1);
#     arc(cr, t.x*ppm + buf*ppm, t.y*ppm + buf*ppm, size, 0, 2*pi);
#     fill(cr);
#     restore(cr)
# end

function draw_target(cr::CairoContext, t::Target, ppm, size, buf)
    save(cr)
    set_source_rgba(cr, 0, 0.3, 0.5, 1)
    root = buf * ppm

    base_angle = t.heading
    tx = t.x * ppm + root
    ty = t.y * ppm + root
    dphi = (2 * pi) / (t.nfaces - 1)
    side_length = 2 * t.apothem * tan(dphi/2)
    # r = ppm * sqrt(t.apothem^2 + (side_length / 2)^2)
    # r = ppm*(t.apothem)/cos(dphi/2)
    r = t.apothem*ppm

    # # Draw heading
    # move_to(cr, tx, ty)
    # set_source_rgba(cr, 0.0, 0.3, 0.6, 1)
    # rel_line_to(cr, ppm * t.apothem * face.normal[1], ppm * t.apothem * face.normal[2])
    # Draw just normals for now
    for (i, face) in enumerate(t.faces[1:length(t.faces)-1])
        set_line_width(cr, 1 + 2 * face.weight)
        save(cr)
        x = face.pos[1] * ppm + root
        y = face.pos[2] * ppm + root
        # move_to(cr, x, y)
        # rel_line_to(cr, ppm*t.apothem * face.normal[1], ppm*t.apothem * face.normal[2])
        theta_offset = dphi/2
        fx_low = tx + r  * cos(base_angle + (i * dphi) - theta_offset)
        fx_high = tx + r * cos(base_angle + (i * dphi) + theta_offset)
        fy_low = ty + r  * sin(base_angle + (i * dphi) - theta_offset)
        fy_high = ty + r * sin(base_angle + (i * dphi) + theta_offset)
        set_source_rgba(cr, 0, 0.3, 0.5, 1)
        stroke(cr)
        move_to(cr, fx_low, fy_low)
        line_to(cr, fx_high, fy_high)
        move_to(cr, x, y)
        # set_source_rgba(cr, 0.8, 0.3, 0.3, 1)
        # set_line_width(cr, 5)
        rel_line_to(cr, ppm * face.normal[1], ppm * face.normal[2])
        restore(cr)
    end
    stroke(cr)
    set_source_rgba(cr, 0, 0.3, 0.5, 1)
    arc(cr, t.x * ppm + buf * ppm, t.y * ppm + buf * ppm, size, 0, 2 * pi)
    fill(cr)
    restore(cr)
end
function draw_targets(cr::CairoContext, targs::Vector{Target}, ppm, size, buf)

    for (i,t) in enumerate(targs)
        cfade = i/length(targs)
        set_source_rgba(cr, (cfade), (1-cfade)*0.5, (1-cfade)*1.1, 1/2);
        draw_target(cr, t,ppm, size, buf)
    end
end

function draw_state(cr::CairoContext, state::UAVState, model, ppm, fade, cfade, buf)
    save(cr)
    move_to(cr,state.x*ppm + buf*ppm, state.y*ppm + buf*ppm)
    fov = model.sensor.fov
    radius = model.sensor.cutoff
    draw_arc(cr, radius, state.x, state.y, state.heading, fov, ppm, fade,cfade, buf)
    restore(cr)
end

function draw_arc(cr::CairoContext, radius, x,y, heading,fov,ppm, fade,cfade, buf)
    ## original example, following here
    xc = x*ppm + buf*ppm;
    yc = y*ppm + buf*ppm;
    radius = radius*ppm;
    angle1 = dirAngle(heading) + (-fov/2);  # angles are specified
    angle2 = dirAngle(heading) + (fov/2);  # in radians

#     set_source_rgba(cr, 0, 0, 0, fade);
    set_source_rgba(cr, (cfade), (1-cfade)*0.5, (1-cfade)*1.1, fade);
    set_line_width(cr, 5.0);
    arc(cr, xc, yc, radius, angle1, angle2);
    fill(cr)
    stroke(cr);

    # draw helping lines
    set_line_width(cr, 6.0);

    # Draw center
    arc(cr, xc, yc, 10.0, 0, 2*pi);
    fill(cr);

    arc(cr, xc, yc, radius, angle1, angle1);
    line_to(cr, xc, yc);

    arc(cr, xc, yc, radius, angle2, angle2);
    line_to(cr, xc, yc);
    close_path(cr);
    set_source_rgba(cr, (cfade), (1-cfade)*0.5, (1-cfade)*1.1, fade/2);
    fill_preserve(cr);

    stroke(cr);
end


function render_paths(solution, multi_configs::MultiDroneMultiActorConfigs)
    paths = Vector{Vector{MDMA.MDPState}}(undef, 0)
    for sol in solution.elements
        push!(paths, sol[2])
    end
    draw_frames(RenderConf(50, 4, false, false), multi_configs, paths)
end

function draw_scene(rconf::RenderConf, model,  paths, cutoff)
    c, cr = init_cairo(model, rconf)
    ppm = rconf.ppm
    buf = rconf.buf
    draw_grid(model.grid, cr, 5, ppm, buf)

    save(cr)
    select_font_face(cr, "Latin Modern Math", Cairo.FONT_SLANT_NORMAL,
        Cairo.FONT_WEIGHT_NORMAL)

    set_font_size(cr, 80.0)
    set_source_rgba(cr, 0, 0, 0, 1)
    move_to(cr, model.grid.width / 2 * ppm + -2 * ppm, model.grid.height * ppm + 1.6 * buf * ppm)
    show_text(cr, "N = $(model.num_robots)  t = $(cutoff)")
    restore(cr)


    for path in paths
        # Draw States
        if rconf.draw_old_states
            state = path[1]
            move_to(cr, state.state.x * ppm + buf * ppm, state.state.y * ppm + buf * ppm)
            draw_state(cr, state.state, model, ppm, 0.4, 1 / cutoff, buf)
            for (i, state) in enumerate(path[2:cutoff-1])
                draw_state(cr, state.state, model, ppm, 0.4, i / cutoff, buf)
            end
        end
        draw_state(cr, path[cutoff].state, model, ppm, 0.4, cutoff / cutoff, buf)

        # Draw Lines
        if rconf.draw_paths
            state = path[1]
            move_to(cr, state.state.x * ppm + buf * ppm, state.state.y * ppm + buf * ppm)
            for (i, state) in enumerate(path[2:cutoff])
                cfade = i / cutoff
                fade = 1
                set_source_rgba(cr, (cfade), (1 - cfade) * 0.5, (1 - cfade) * 1.1, fade / 2)
                set_line_width(cr, 13.0)
                move_to(cr, path[i].state.x * ppm + buf * ppm, path[i].state.y * ppm + buf * ppm)
                line_to(cr, state.state.x * ppm + buf * ppm, state.state.y * ppm + buf * ppm)
                stroke(cr)
            end
        end
    end

    targets = model.target_trajectories[cutoff, :]
    draw_targets(cr, targets, ppm, 15, buf)
    filepath = "output/$(model.experiment_name)/renders/$(lpad(cutoff, 2, "0")).png"
    println("Writing to ", filepath)
    write_to_png(c, filepath)

end

#Pixels per meter
function init_cairo(model,conf::RenderConf)
    width = conf.ppm*dims(model.grid)[1]+ 2*conf.buf*conf.ppm
    height = conf.ppm*dims(model.grid)[2]+ 2*conf.buf*conf.ppm
    c = CairoRGBSurface(width,height);
    cr = CairoContext(c);
    draw_background(cr, width, height)
    return (c, cr)
end

function draw_frames(rconf::RenderConf, model, paths)
  for i in 1:model.horizon
      draw_scene(rconf, model, paths, i)
  end
end

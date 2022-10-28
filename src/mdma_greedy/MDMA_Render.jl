using Cairo

struct RenderConf
    ppm::Float64
    buf::Int64
end



function draw_background(cr::CairoContext, width, height)
    save(cr);
    set_source_rgb(cr,1,1,1);    # white
    rectangle(cr,0.0,0.0,width,height); # background
    fill(cr);
    restore(cr);
end

function draw_grid(g::Grid, cr::CairoContext, point_size, ppm, buf)
    set_source_rgba(cr, 0, 0, 0, 0.3);
    x,y,z = dims(g)
    print(dims(g))
    for i in 1:x
        for j in 1:y
            arc(cr, i*ppm + buf*ppm, j*ppm+ buf*ppm, point_size, 0, 2*pi);
            fill(cr);
        end
    end
end

function draw_target(cr::CairoContext, t::Target, ppm, size, buf)
    save(cr)
    set_source_rgba(cr, 0, 0.3, 0.5, 1);
    arc(cr, t.x*ppm + buf*ppm, t.y*ppm + buf*ppm, size, 0, 2*pi);
    fill(cr);
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


function draw_path(model, path, cutoff)
    rconf = RenderConf(50, 4)
    c, cr = init_cairo(model, rconf)
    ppm = rconf.ppm
    buf = rconf.buf
    draw_grid(model.grid, cr, 5, ppm, buf)

    save(cr)
    select_font_face(cr, "Latin Modern Math", Cairo.FONT_SLANT_NORMAL,
                 Cairo.FONT_WEIGHT_NORMAL);

    set_font_size(cr, 80.0);
    set_source_rgba(cr, 0, 0, 0, 1);
    move_to(cr, model.grid.width/2*ppm + -2*ppm, model.grid.height*ppm + 1.6*buf*ppm);
    show_text(cr, "𝐷 = $(model.move_dist)  t = $(cutoff)");
    restore(cr)


    # Draw States
    state=path[1]
    move_to(cr,state.state.x*ppm + buf*ppm, state.state.y*ppm + buf*ppm)
    draw_state(cr, state.state, model, ppm, 0.4, 1/cutoff, buf)
    for (i,state) in enumerate(path[2:cutoff])
        draw_state(cr, state.state, model, ppm, 0.4, i/cutoff, buf)
    end

    # Draw Lines
    state=path[1]
    move_to(cr,state.state.x*ppm + buf*ppm, state.state.y*ppm + buf*ppm)
    for (i,state) in enumerate(path[2:cutoff])
        cfade = i/cutoff
        fade=1
        set_source_rgba(cr, (cfade), (1-cfade)*0.5, (1-cfade)*1.1, fade/2);
        set_line_width(cr, 13.0);
        move_to(cr,path[i].state.x*ppm+ buf*ppm, path[i].state.y*ppm + buf*ppm)
        line_to(cr, state.state.x*ppm+buf*ppm, state.state.y*ppm + buf*ppm)
        stroke(cr)


    end
    targets = model.target_trajectories[cutoff,:]
    draw_targets(cr, targets,ppm, 15, buf)

    write_to_png(c,"anim/$(lpad(cutoff, 2, "0")).png");

end

#Pixels per meter
function init_cairo(model,conf::renderconf)
    width = conf.ppm*dims(model.grid)[1]+ 2*conf.buf*conf.ppm
    height = conf.ppm*dims(model.grid)[2]+ 2*conf.buf*conf.ppm
    c = cairorgbsurface(width,height);
    cr = cairocontext(c);
    draw_background(cr, width, height)
    return (c, cr)
end

function draw_animated(model, path, cutoff)
    rconf = RenderConf(50, 4)
    c, cr = init_cairo(model, rconf)
    ppm = rconf.ppm
    buf = rconf.buf
    draw_grid(model.grid, cr, 5, ppm, buf)

    save(cr)
    select_font_face(cr, "Latin Modern Math", Cairo.FONT_SLANT_NORMAL,
                 Cairo.FONT_WEIGHT_NORMAL);

    set_font_size(cr, 80.0);
    set_source_rgba(cr, 0, 0, 0, 1);
    move_to(cr, model.grid.width/2*ppm + -2*ppm, model.grid.height*ppm + 1.6*buf*ppm);
    show_text(cr, "𝐷 = $(model.move_dist)  t = $(cutoff)");
    restore(cr)

    # Draw States
    state=path[cutoff]
    move_to(cr,state.state.x*ppm + buf*ppm, state.state.y*ppm + buf*ppm)
    draw_state(cr, state.state, model, ppm, 0.4, 1/cutoff, buf)

    targets = model.target_trajectories[cutoff,:]
    draw_targets(cr, targets,ppm, 15, buf)

    write_to_png(c,"anim/$(lpad(cutoff, 2, "0")).png");

end

function compute_path(model, policy, state)
    path = Vector{MDPState}(undef, 0)
    push!(path, state)
    for x in 2:model.horizon
        state = action(policy, state);
        #         state = MDPState(state);
        push!(path, state);
    end
    path
end
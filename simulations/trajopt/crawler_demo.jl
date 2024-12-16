using Pkg
Pkg.activate(joinpath(@__DIR__, "../examples/"))
Pkg.add("MAT")
Pkg.instantiate()

# ## Setup
using Dojo
using Random
using LinearAlgebra 
using JLD2
using StaticArrays
using IterativeLQR
import Dojo: cost
using Dates
using MAT
#include(joinpath(@__DIR__, "algorithms/ags.jl")) # augmented random search
include(joinpath(@__DIR__, "../environments/crawler/methods/initialize.jl"))
include(joinpath(@__DIR__, "../environments/crawler/methods/env.jl"))

case = 1
case_name = ["distal"; "brittlestar"; "robot"; "central"]
case_mult = [1.8; 1.375; 1; 0.25]
# ## Patrick
# set attributes of this robot
num_limbs = 5 # wouldn't recommend doing less than 3 for now. lemme know if you want less
num_links_per_limb = 2
num_actuated_per_limb = 1
mass_total = 0.33*100
mass_per_limb = 0.036*case_mult[case] #*1.375 for brittle star, *0.25 for central, *1.8 for distal
mass_body = mass_total - mass_per_limb*5
radius_b = 0.045
radius = 0.1
limb_length = 0.13*5
gravity=[0.0, 0.0, -9.81]
timestep=0.05

# this call produces the simulation environment (mechanism, actuation template, animations, dynamics)
env = crawler(
    representation=:minimal, 
    gravity=gravity, 
    timestep=timestep, 
    damper=0.1, 
    spring=0.0, 
    friction_coefficient=0.1,
    contact_body=true,
    num_links=2,
    num_u=1,
    radius=radius,
    radius_b=radius_b,
    height=limb_length,
    limits=false);

obs = reset(env)
env.state .= get_minimal_state(env.mechanism)
render(env)

# initialize state and open the visualizer
body_position=[0.0; 0.0; radius*10 + radius/100]
# ## Open visualizer
initialize_crawler!(env.mechanism,body_position=body_position,body_orientation=[0.0; π; 0.0])
open(env.vis)



body_position=[0.0; 0.0; radius + radius/100]
# set our current state and initialize y to store our trajectory
initialize_crawler!(env.mechanism,body_position=body_position,body_orientation=[0.0; π/2; 0.0 * π])
env.state .= get_minimal_state(env.mechanism)
y = [copy(env.state)] # state trajectory
T = 101

# initialize our inputs (these are joint impulses)
ū = [zeros(1) for i in 1:T-1]
x_traj = LinRange(0, 2*π - 0.0001, 100)
#ū .= u_sol_old
# perform a rollout. these inputs are just me playing around by hand
for t = 1:50
    ū[t] = [0.4*(x_traj[t] - y[end][13])] #[1.127*(1-t/50.0)]
    # ū[t] = [1.127*(1-t/50.0)]
    step(env, env.state, ū[t])
    push!(y, copy(env.state)) 
end
# for t = 51:100
#     ū[t] = [0.0]
#     step(env, env.state, ū[t])
#     push!(y, copy(env.state)) 
# end

# visualize our rollout
visualize(env, y[1:end]);




env.state .= get_minimal_state(env.mechanism);

y = [copy(env.state)]
# initialize our inputs (these are joint impulses)
ū = [zeros(env.num_inputs) for i in 1:T-1]
#ū .= u_sol_old
# perform a rollout. these inputs are just me playing around by hand
for t = 1:25
    ū[t] = [zeros(4); -1.3; 1.2; zeros(env.num_inputs-10); -1.3; -1.2; zeros(2)]/2000
    step(env, env.state, ū[t])
    push!(y, copy(env.state)) 
end
for t = 26:50
    ū[t] = 1/5*[zeros(4); 3; -6; zeros(env.num_inputs-10); 3; 6; zeros(2)]/2000
    #ū[t] = 1/5*[zeros(4); 6; -5; zeros(env.num_inputs-10); 6; 5; zeros(2)]/2000
    step(env, env.state, ū[t])
    push!(y, copy(env.state)) 
end
visualize(env, y[1:3]);






initialize_patrick!(env.mechanism,body_position=body_position,body_orientation=[0.0; 0.0; 0.0 * π])
env.state .= get_minimal_state(env.mechanism);
y = [copy(env.state)]
contact_force_storage = [copy([collect(contact.impulses[2]) for contact in env.mechanism.contacts])]
#env.state .= get_minimal_state(env.mechanism)
reflect = Diagonal(repeat([1;-1; 1; -1],num_limbs))
for i=1:4
    for t = 1:12
        u = [zeros(4); -1.5; -0.2; zeros(env.num_inputs-10); -1.5; 0.2; zeros(2)]/2000
        
        step(env, env.state, u)
        push!(y, copy(env.state)) 
        push!(contact_force_storage, copy([collect(contact.impulses[2]) for contact in env.mechanism.contacts])) 
    end
    for t = 13:25
        u = [zeros(4); -0.4; 1.5; zeros(env.num_inputs-10); -0.4; -1.5; zeros(2)]/2000
        step(env, env.state, u)
        push!(y, copy(env.state)) 
        push!(contact_force_storage, copy([collect(contact.impulses[2]) for contact in env.mechanism.contacts])) 
    end
    for t = 26:37
        u = 1/5*[zeros(4); 3; 4; zeros(env.num_inputs-10); 3; -4; zeros(2)]/2000
        step(env, env.state, u)
        push!(y, copy(env.state)) 
        push!(contact_force_storage, copy([collect(contact.impulses[2]) for contact in env.mechanism.contacts])) 
    end
    for t = 38:50
        u = 1/5*[zeros(4); 3; -9; zeros(env.num_inputs-10); 3; 9; zeros(2)]/2000
        step(env, env.state, u)
        push!(y, copy(env.state)) 
        push!(contact_force_storage, copy([collect(contact.impulses[2]) for contact in env.mechanism.contacts])) 
    end
end
visualize(env, y);


storage = generate_storage(env.mechanism, [env.representation == :minimal ? minimal_to_maximal(env.mechanism, x) : x for x in y]);
# visualize(env.mechanism, storage,
#     vis=env.vis, show_contact = true);

res = get_sdf(env.mechanism, storage)
path = "echino-sim/trajopt/results/"
# @save joinpath(@__DIR__, "results/patrick_contacts_ilqr-2022-05-24 15:25:02.jld2") res
file = matopen(path*"patrick_contacts_nominal_"*case_name[case]*".mat", "w")
write(file, "contacts", res)
close(file)
file = matopen(path*"patrick_minimal_state_nominal_"*case_name[case]*".mat", "w")
write(file, "state", y)
close(file)
file = matopen(path*"patrick_maximal_state_nominal_"*case_name[case]*".mat", "w")
write(file, "state", [[collect(j) for j in i] for i in storage.x])
close(file)
file = matopen(path*"patrick_contact_forces_nominal_"*case_name[case]*".mat", "w")
write(file, "contacts", contact_force_storage)
close(file)






### BELOW IS AN EXAMPLE OF TRAJECTORY OPTIMIZATION.
# We take the above inputs, perform a rollout, and use that to initialize our iLQR problem. 
# If we make a good choice of cost function, our solution converges to a biomimetic gait

@load joinpath(@__DIR__, "results/patrick_ilqr_distal-2022-06-25 18:51:22.jld2") x_sol u_sol
ū .= u_sol;

# ## dimensions
n = env.num_states
m = env.num_inputs

# ## model
dyn = IterativeLQR.Dynamics(
    (y, x, u, w) -> dynamics(y, env, x, u, w),
    (dx, x, u, w) -> dynamics_jacobian_state(dx, env, x, u, w),
    (du, x, u, w) -> dynamics_jacobian_input(du, env, x, u, w),
    n, n, m)

model = [dyn for t = 1:T-1]

# ## rollout
#initialize_patrick!(env.mechanism,body_position=body_position,body_orientation=[0.0; 0.0; 0.0 * π])
x1 = get_minimal_state(env.mechanism);
# x1 = x_sol[1];
x̄ = IterativeLQR.rollout(model, x1, ū)
visualize(env, x̄);
xref = [[-1.0*t/50; 0.0; height; zeros(3); -0.5; zeros(5); x1[13:end]] for t=0:T-1]
# xref .= x_sol_old
timestep=0.05

# ## objective
# The cost function we choose induces the robot to:
# Move forward (in the -x direction),
# Keep the body close to the ground in the z (to prevent jumping) 
# Maintain forward (-x) velocity as much as possible, 
# End with the joints in the same state as the start,
# Use minimal inputs to accomplish the above
# a critical factor to tune performance is relative_input_cost, which determines how aggressive inputs should be
relative_input_cost = 420000 #50
qt = [50.0; 0.0; 100; zeros(3); 10.0; zeros(5); zeros(length(env.state)-12)]
# qt = [50*ones(12); 50*ones(length(env.state)-12)]
qt_end = [10.0; 0.0; 100; 1.0*ones(3); 10.0; zeros(5); 100*ones(length(env.state)-12)]
# qt_end = [50*ones(12); 50*ones(length(env.state)-12)]
rt = repeat([1; 1; 1000; 1000], num_limbs)
ots = [(x, u, w) -> transpose(x - xref[t]) * Diagonal(timestep * qt) * (x - xref[t]) + relative_input_cost * transpose(u) * Diagonal(timestep * ones(m)) * u for t = 1:T-1]
oT = (x, u, w) -> 5*transpose(x - xref[end]) * Diagonal(timestep * qt_end) * (x - xref[end])

# These statements can be used to add constraints to the optimizer
# Note this likely adds to computational cost because it extends the solution from a single iLQR
# solution to add an outer augmented lagrangian loop
cts = [IterativeLQR.Cost(ot, n, m) for ot in ots]
cT = IterativeLQR.Cost(oT, n, 0)
obj = [cts..., cT]

# ## constraints
# this can be used to set a goal state
function goal(x, u, w)
    Δ = x - xref
    return Δ[collect(1:3)]
end

cont = IterativeLQR.Constraint()
# conT = IterativeLQR.Constraint(goal, n, 0)
cons = [[cont for t = 1:T-1]..., cont]

# ## solver
s = IterativeLQR.solver(model, obj, cons, 
    opts=IterativeLQR.Options(
        max_al_iter=10,
        verbose=true))

IterativeLQR.initialize_controls!(s, ū)
IterativeLQR.initialize_states!(s, x̄)

# ## solve
@time IterativeLQR.solve!(s)

# ## solution
x_sol, u_sol = IterativeLQR.get_trajectory(s)
@show IterativeLQR.eval_obj(s.m_data.obj.costs, s.m_data.x, s.m_data.u, s.m_data.w)
@show s.s_data.iter[1]

# ## visualize
vis= Visualizer()
open(env.vis)
visualize(env, x_sol);


body_position=[0.0; 0.0; height/2 + height/100]
# this is a little snippet that takes an optimal gait and repeats it (in alternating fashion)
initialize_patrick!(env.mechanism,body_position=body_position,body_orientation=[0.0; 0.0; 0.0 * π])
env.state .= get_minimal_state(env.mechanism);
env.state = init_state
y = [copy(env.state)]

#env.state .= get_minimal_state(env.mechanism)
reflect = Diagonal(repeat([1;-1; 1; -1],num_limbs))
for i=1:4
    for t = 1:50
        if i % 2 == 0
            u = reflect*[u_sol[t][1:4]; u_sol[t][17:end]; u_sol[t][13:16]; u_sol[t][9:12]; u_sol[t][5:8]]
            u = reflect*[u_sol[t][1:4]; u_sol[t][17:end]; u_sol[t][13:16]; u_sol[t][9:12]; u_sol[t][5:8]]
        else
            u = u_sol[t]
        end
        step(env, env.state, u)
        push!(y, copy(env.state)) 
        #push!(contact_force_storage, copy([collect(contact.impulses[2]) for contact in env.mechanism.contacts])) 

    end
end
visualize(env, y);
y = x_sol;



storage = generate_storage(env.mechanism, [env.representation == :minimal ? minimal_to_maximal(env.mechanism, x) : x for x in y]);
# visualize(env.mechanism, storage,
#     vis=env.vis, show_contact = true);

res = get_sdf(env.mechanism, storage)
path = "echino-sim/trajopt/results/"
# @save joinpath(@__DIR__, "results/patrick_contacts_ilqr-2022-05-24 15:25:02.jld2") res
file = matopen(path*"patrick_contacts_distr_"*case_name[case]*".mat", "w")
write(file, "contacts", res)
close(file)
file = matopen(path*"patrick_minimal_state_distr_"*case_name[case]*".mat", "w")
write(file, "state", y)
close(file)
file = matopen(path*"patrick_maximal_state_distr_"*case_name[case]*".mat", "w")
write(file, "state", [[collect(j) for j in i] for i in storage.x])
close(file)
file = matopen(path*"patrick_actuators_distr_"*case_name[case]*".mat", "w")
write(file, "inputs", u_sol)
close(file)
file = matopen(path*"patrick_contact_forces_12_"*case_name[case]*".mat", "w")
write(file, "contacts", contact_force_storage)
close(file)

# this is stuff for post processing data
@save joinpath(@__DIR__, "results/patrick_ilqr_distr_"*case_name[case]*"-"*Dates.format(now(), "yyyy-mm-dd HH:MM:SS")*".jld2") x_sol u_sol
@save joinpath(@__DIR__, "results/patrick_distal_legs_0.009_body_0.285_ilqr-"*Dates.format(now(), "yyyy-mm-dd HH:MM:SS")*".jld2") y

@load joinpath(@__DIR__, "results/patrick_ilqr-2022-06-18 14:22:13.jld2") x_sol u_sol

@load joinpath(@__DIR__, "results/patrick_ilqr_central-2022-06-21 11:34:41.jld2") x_sol u_sol

y = [copy(x_sol[1])]
contact_force_storage = [copy([collect(contact.impulses[2]) for contact in env.mechanism.contacts])]
for t = 1:50
    u = u_sol[t]
    step(env, env.state, u)
    push!(y, copy(env.state)) 
    push!(contact_force_storage, copy([collect(contact.impulses[2]) for contact in env.mechanism.contacts])) 
end

searchdir(path,key) = filter(x->contains(x,key), readdir(path))

key = "patrick_y"
for name in searchdir(path,key)
    @load joinpath(@__DIR__, "results/"*name) y
    @show name[11:end-30]
    file = matopen(path*name[11:end-30]*".mat", "w")
    write(file, "state", y)
    close(file)
end

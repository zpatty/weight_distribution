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
using DelimitedFiles
#include(joinpath(@__DIR__, "algorithms/ags.jl")) # augmented random search
include(joinpath(@__DIR__, "../environments/patrick/methods/initialize.jl"))
include(joinpath(@__DIR__, "../environments/patrick/methods/env.jl"))

case = 5
mass_in_limbs_names = ["distal"; "brittlestar"; "robot"; "central"]
mass_in_limbs = [0.98; 0.75; 0.55; 0.15]
mass_proximal = [0.5; 0.75; 0.95]
masses = LinRange(0.33/2,0.33*2,5)
lengths = LinRange(0.13/2,0.13*2,5)

num_limbs = 5 # wouldn't recommend doing less than 3 for now. lemme know if you want less
num_links_per_limb = 2
num_actuated_per_limb = 2

radius_b = 0.045
height = 0.032
body_position=[0.0; 0.0; height/2 + height/100]

gravity=[0.0, 0.0, -2.81]
timestep=0.05

T = 51
environments = Vector{Any}(undef,300)
m = 1
for (i,j,k,l) in Iterators.product(masses,lengths, mass_proximal, mass_in_limbs)
    mass_total = i
    limb_length = j
    mass_body = mass_total * (1 - l)
    mass_per_limb = mass_total * l / 5 #*1.375 for brittle star, *0.25 for central, *1.8 for distal
    
    environments[m] = patrick(
        representation=:minimal, 
        gravity=gravity, 
        timestep=timestep, 
        damper=0.10, 
        spring=0.005*ones(num_links_per_limb*num_limbs), 
        friction_coefficient=1.0,
        contact_feet=false, 
        contact_body=true,
        num_limbs=num_limbs,
        num_links_per_limb=num_links_per_limb,
        num_actuated_per_limb=num_actuated_per_limb,
        mass_per_limb=mass_per_limb,
        mass_body=mass_body,
        radius_b = radius_b,
        height=height,
        limb_length=limb_length,
        mass_proximal=k,
        limits=false,
        joint_limits=[-50*ones(num_links_per_limb*5*2)* π / 180.0,
                       50*ones(num_links_per_limb*5*2)* π / 180.0]);
    
    # println("mass_proximal = $k, mass_in_limbs = $l")
    obs = reset(environments[m]);
    # environments[m].state .= get_minimal_state(environments[m].mechanism);
    # render(environments[m]);
    m = m+1;
end

@load joinpath(@__DIR__, "results/patrick_ilqr-2022-06-18 14:22:13.jld2") x_sol u_sol
ū = [zeros(environments[1].num_inputs) for i in 1:T-1]
u_sol_start = u_sol;
ū .= u_sol_start;


n = environments[1].num_states
m = environments[1].num_inputs

xref = [[-1.0*t/25; 0.0; height; zeros(3); -0.5; zeros(5); x_sol[t+1][13:end]] for t=0:T-1]
# xref .= x_sol_old
timestep=0.05

relative_input_cost = 420000 #50
qt = [50.0; 0.0; 10000; zeros(3); 10.0; zeros(5); zeros(length(environments[1].state)-12)]
# qt = [50*ones(12); 50*ones(length(env.state)-12)]
qt_end = [100.0; 0.0; 1000; 1.0*ones(3); 10.0; zeros(5); 100*ones(length(environments[1].state)-12)]
# qt_end = [50*ones(12); 50*ones(length(env.state)-12)]
rt = repeat([1; 1; 1000; 1000], num_limbs)
ots = [(x, u, w) -> transpose(x - xref[t]) * Diagonal(timestep * qt) * (x - xref[t]) + relative_input_cost * transpose(u) * Diagonal(timestep * ones(m)) * u for t = 1:T-1]
oT = (x, u, w) -> 5*transpose(x - xref[end]) * Diagonal(timestep * qt_end) * (x - xref[end])

function goal(x, u, w)
    Δ = x - xref
    return Δ[collect(1:3)]
end

function fitness(model)
    dyn = IterativeLQR.Dynamics(
    (y, x, u, w) -> dynamics(y, env, x, u, w),
    (dx, x, u, w) -> dynamics_jacobian_state(dx, env, x, u, w),
    (du, x, u, w) -> dynamics_jacobian_input(du, env, x, u, w),
    n, n, m)

    dynamics = [dyn for t = 1:T-1]

    # ## rollout
    initialize_patrick!(model.env.mechanism,body_position=body_position,body_orientation=[0.0; 0.0; 0.0 * π])
    x1 = get_minimal_state(model.env.mechanism);
    # x1 = x_sol[1];
    x̄ = IterativeLQR.rollout(dynamics, x1, ū)
    visualize(model.env, x̄);

    cts = [IterativeLQR.Cost(ot, n, m) for ot in ots]
    cT = IterativeLQR.Cost(oT, n, 0)
    obj = [cts..., cT]

    cont = IterativeLQR.Constraint()
    # conT = IterativeLQR.Constraint(goal, n, 0)
    cons = [[cont for t = 1:T-1]..., cont]

    # ## solver
    s = IterativeLQR.solver(dynamics, obj, cons, 
    opts=IterativeLQR.Options(
        max_al_iter=10,
        verbose=false))

    IterativeLQR.initialize_controls!(s, ū)
    IterativeLQR.initialize_states!(s, x̄)

    # ## solve
    # @time IterativeLQR.solve!(s)

    # ## solution
    x_sol, u_sol = IterativeLQR.get_trajectory(s)

    return x_sol[end][1]^2 * 10 - sum([x[3]^2] for x in x_sol) - sum([u'*u for u in u_sol])
end

function crossover(m1,m2)
    params1 = m1.params
    params2 = m2.params
    n1, n2 = AX(params1, params2)
    return Patrick(n1), Patrick(n2)
end

function gaussian_pat(σ::Real = 1.0)
    vop = gaussian(σ)
    function mutation(model::T) where {T <: Patrick}  
        θ = model.params
        return Patrick(convert(Vector{Float32}, vop(θ)))
    end
    return mutation
end

import Evolutionary.initial_population
function initial_population(method::M, individual::Patrick;
                            kwargs...) where {M<:Evolutionary.AbstractOptimizer}
    params = [rand()*(0.33*2 - 0.33/2); rand()*(0.13*2 - 0.13/2); rand(); rand()]
    [Patrick(params) for i in 1:Evolutionary.population_size(method)]
end

import Evolutionary.EvolutionaryObjective
function EvolutionaryObjective(f, x::Patrick; eval::Symbol = :serial)
    fval = f(x)
    EvolutionaryObjective{typeof(f),typeof(fval),typeof(x),Val{eval}}(f, fval, deepcopy(x), 0)
end

x_results = zeros(length(environments))
Threads.@threads for i=1:length(environments)
    env = environments[i]
    dyn = IterativeLQR.Dynamics(
    (y, x, u, w) -> dynamics(y, env, x, u, w),
    (dx, x, u, w) -> dynamics_jacobian_state(dx, env, x, u, w),
    (du, x, u, w) -> dynamics_jacobian_input(du, env, x, u, w),
    n, n, m)

    model = [dyn for t = 1:T-1]

    # ## rollout
    initialize_patrick!(env.mechanism,body_position=body_position,body_orientation=[0.0; 0.0; 0.0 * π])
    x1 = get_minimal_state(env.mechanism);
    # x1 = x_sol[1];
    x̄ = IterativeLQR.rollout(model, x1, ū)
    visualize(env, x̄);

    cts = [IterativeLQR.Cost(ot, n, m) for ot in ots]
    cT = IterativeLQR.Cost(oT, n, 0)
    obj = [cts..., cT]

    cont = IterativeLQR.Constraint()
    # conT = IterativeLQR.Constraint(goal, n, 0)
    cons = [[cont for t = 1:T-1]..., cont]

    # ## solver
    s = IterativeLQR.solver(model, obj, cons, 
    opts=IterativeLQR.Options(
        max_al_iter=10,
        verbose=false))

    IterativeLQR.initialize_controls!(s, ū)
    IterativeLQR.initialize_states!(s, x̄)

    # ## solve
    # @time IterativeLQR.solve!(s)

    # ## solution
    x_sol, u_sol = IterativeLQR.get_trajectory(s)
    # @show IterativeLQR.eval_obj(s.m_data.obj.costs, s.m_data.x, s.m_data.u, s.m_data.w)
    # @show s.s_data.iter[1]
    
    @save joinpath(@__DIR__, "results/patrick_ilqr_distr_"*string(i)*"-"*Dates.format(now(), "yyyy-mm-dd HH:MM:SS")*".jld2") x_sol u_sol

    x_results[i] = x_sol[end][1:3]
end

idx = 1
mass_vec = zeros(300)
l_vec = zeros(300)
prox_vec = zeros(300)
limb_vec = zeros(300)
for (i,j,k,l) in Iterators.product(masses,lengths, mass_proximal, mass_in_limbs)
    mass_vec[idx] = i
    l_vec[idx] = j
    prox_vec[idx] = k
    limb_vec[idx] = l
    idx = idx+1
end

writedlm( "x_results.csv",  x_results, ',')

x_results = zeros(length(environments),3)
for i =1:300
    @load joinpath(@__DIR__, "results/param_sweep/"*string(i)*".jld2") x_sol u_sol
    x_results[i,1:3] = x_sol[end][1:3]
end
writedlm( "x_results.csv",  x_results, ',')

### BELOW IS AN EXAMPLE OF TRAJECTORY OPTIMIZATION.
# We take the above inputs, perform a rollout, and use that to initialize our iLQR problem. 
# If we make a good choice of cost function, our solution converges to a biomimetic gait




# ## dimensions

# ## model


# ## objective
# The cost function we choose induces the robot to:
# Move forward (in the -x direction),
# Keep the body close to the ground in the z (to prevent jumping) 
# Maintain forward (-x) velocity as much as possible, 
# End with the joints in the same state as the start,
# Use minimal inputs to accomplish the above
# a critical factor to tune performance is relative_input_cost, which determines how aggressive inputs should be


# These statements can be used to add constraints to the optimizer
# Note this likely adds to computational cost because it extends the solution from a single iLQR
# solution to add an outer augmented lagrangian loop


# ## constraints
# this can be used to set a goal state






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

@load joinpath(@__DIR__, "results/patrick_ilqr_distr_long_dist-2023-05-19 11:00:49.jld2") x_sol u_sol

@load joinpath(@__DIR__, "results/patrick_ilqr_distr_268-2023-05-30 14:24:43.jld2") x_sol u_sol

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

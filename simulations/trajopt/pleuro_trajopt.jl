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
using Dates
using MAT
import Dojo: cost



#include(joinpath(@__DIR__, "algorithms/ags.jl")) # augmented random search
include(joinpath(@__DIR__, "../environments/pleuro/methods/initialize.jl"))
include(joinpath(@__DIR__, "../environments/pleuro/methods/env.jl"))
# ## Ant
num_links = 4
num_u = 2
timestep=0.05
scale=1
factor = 20/scale
radius = 0.01*scale
radius_b = 0.05*scale
h_total = 0.15*scale
height = h_total/num_links
height_b = 2*radius
mass = π*radius^2*height*1500
mass_2 = mass/2
mass_b = π*radius_b^2*height_b*1500
mass_total = (mass*num_u + mass_2*(num_links-num_u) + mass_b)
Fg = 9.81*mass_total
V_b = π*radius^2*height*num_links + π*radius_b^2*height_b
Fb = V_b*1000*9.81
g = (Fg-Fb)/mass_total
@show g
gravity=[0.0; 0.0; -g]
env = pleuro(
    representation=:minimal, 
    gravity=gravity, 
    timestep=timestep, 
    damper=0.1, 
    spring=[1.0/1000*scale*ones(2); scale*ones(2)], 
    friction_coefficient=0.5,
    contact_feet=true, 
    contact_body=true,
    num_links=num_links,
    num_u=num_u,
    radius=radius,
    radius_b=radius_b,
    height=height,
    limits=true);

obs = reset(env)
# initialize!(env.mechanism, Dojo.type2symbol(Patrick),
#     body_position=[0.0, 0.0, 1.0], 
#     body_orientation=[0.0, 0.0, 0.0],
#     ankle_orientation=0.0)
# initialize_patrick!(env.mechanism,
#     body_position=[0.0, 0.0, 0.205], 
#     body_orientation=[0.0, 0.0, 0.0],
#     ankle_orientation=0.0)
env.state .= get_minimal_state(env.mechanism)
render(env)
# ## Open visualizer
# initialize!(env.mechanism, Dojo.type2symbol(Patrick))
body_position=[0.0; 0.0; radius + 0.1*radius]
initialize_pleuro!(env.mechanism,body_position=body_position)
open(env.vis)



initialize_pleuro!(env.mechanism,body_position=body_position)
env.state .= get_minimal_state(env.mechanism)
# randn(env.num_inputs)
y = [copy(env.state)] # state trajectory
T = 101
ū = [zeros(env.num_inputs) for i in 1:T-1]
for t = 1:50
    # ū[t] .= 5/100000*scale*ones(num_u)*sin(t/100*2*π)
    ū[t] .= 6/100000*scale*ones(num_u)
    # ū[t][1] = 4/100000*scale*sin(t/80*2*π)
    # ū[t][2:3] = -2/100000*scale*ones(num_links-1)*sin(t/80*2*π)
    # ū[t][1] = 1/200000*scale*sin(t/40*2*π)
    # ū[t][2] = 1/100000*scale*sin(t/40*2*π+1π/3)
    #ū[t][3] = 1/100000*scale*sin(t/40*2*π+2π/3)
    #ū[t] = 0.000006*ones(num_links)*0.0
    step(env, env.state, ū[t])
    push!(y, copy(env.state)) 
end
for t = 51:100
    ū[t] .= -6/100000*scale*ones(num_u)
    # ū[t][1] = 4/100000*scale*sin(t/80*2*π)
    # ū[t][2:3] = -2/100000*scale*ones(num_links-1)*sin(t/80*2*π)
    # ū[t][1] = 4/100000*scale*sin(t/80*2*π)
    # ū[t][2:3] = -2/100000*scale*ones(num_links-1)*sin(t/80*2*π)
    #ū[t] = 0.000006*ones(num_links)*0.0
    step(env, env.state, ū[t])
    push!(y, copy(env.state)) 
end

ul = -1/10000*scale*ones(num_u)
uu = 1/10000*scale*ones(num_u)
# for t = 1:100   
#     ū[t] = 2/100*scale*ones(num_links)
#     step(env, env.state, ū[t])
#     push!(y, copy(env.state)) 
# end
# sinusoidal
# ū[t][1] = 0.2/3*sin(t/80*2*π)
# ū[t][2:3] = -0.3/3*ones(2)*sin(t/80*2*π)

visualize(env, y[1:end]);








# ## dimensions
n = env.num_states
m = env.num_inputs

# ## reference trajectory
#N = 2
#visualize(env, xref)

# storage = simulate!(mech, 1.0,
#     record=true,
#     verbose=false)

# visualize(mech, storage,
#     vis=env.vis)
# ## horizon


# ## model
dyn = IterativeLQR.Dynamics(
    (y, x, u, w) -> dynamics(y, env, x, u, w),
    (dx, x, u, w) -> dynamics_jacobian_state(dx, env, x, u, w),
    (du, x, u, w) -> dynamics_jacobian_input(du, env, x, u, w),
    n, n, m)

model = [dyn for t = 1:T-1]

# ## rollout
#initialize_pleuro!(env.mechanism,body_position=body_position)
x1 = get_minimal_state(env.mechanism)
x̄ = IterativeLQR.rollout(model, x1, ū)
visualize(env, x̄);
# xgoal = [-0.8; 0.0; 0.2; zeros(3); -0.5; zeros(5); 0.7; -0.52; -0.3; -0.5; 0.08; 0.04; 0.02; 0.002]
# xgoal = [-1.0; 0.0; radius; zeros(3); -0.5; zeros(5); 0.5235997717304138;
# 4.632832888429317e-6;
# -0.1831888197137249;
# -0.49900709106006313;
# -0.2888118773615937;
# -0.5164038499880537;
# 0.16204649911700442;
# 0.06905133847779053;
# 0.08347651824670131;
# 0.02591559035806034;
# 0.026400856491096723;
# 0.0016440154564128688]
timestep=0.05

xref = [[-1.0*t/100; 0.0; radius; zeros(3); -0.1; zeros(5); x1[13:end]] for t=0:T-1]
rel_c = 1000000
# ## objective
# qt = [1.0; 0.05; 1.0; 0.01 * ones(3); 0.5; 0.01 * ones(2); 0.01 * ones(3); fill([0.000000000001, 0.0000000000001], num_links-u*2)...]
qt = [10.0; 0.0; 100; zeros(3); 10.0; zeros(5); zeros(num_links*2)]
qt_end = [10.0; 0.0; 100; 1.0*ones(3); 10.0; zeros(5); 100*ones(num_links*2)]

ots = [(x, u, w) -> transpose(x-xref[t]) * Diagonal(timestep * qt) * (x-xref[t]) + rel_c*transpose(u) * Diagonal(timestep * ones(m)) * u for t = 1:T-1]
oT = (x, u, w) -> transpose(x-xref[end]) * Diagonal(timestep * qt_end) * (x-xref[end])

cts = [IterativeLQR.Cost(ot, n, m) for ot in ots]
cT = IterativeLQR.Cost(oT, n, 0)
obj = [cts..., cT]

# ## constraints
function goal(x, u, w)
    Δ = x - xref
    return zeros(3)
end

function limits(x,u,w)
    [
        (ul-u)*10;
        (u-uu)*10;
    ]
end

cont = IterativeLQR.Constraint(limits,n,m,idx_ineq=collect(1:2m))
cont = IterativeLQR.Constraint()
# conT = IterativeLQR.Constraint(goal, n, 0)
cons = [cont for t = 1:T]

# ## solver
s = IterativeLQR.solver(model, obj, cons, 
    opts=IterativeLQR.Options(
        max_al_iter=10,
        verbose=true))
# s = IterativeLQR.solver(model, obj, cons,
#     opts=IterativeLQR.Options(
#         verbose=true,
#         linesearch=:armijo,
#         α_min=1.0e-5,
#         obj_tol=1.0e-3,
#         grad_tol=1.0e-3,
#         max_iter=100,
#         max_al_iter=5,
#         ρ_init=1.0,
#         ρ_scale=10.0))
IterativeLQR.initialize_controls!(s, ū)
IterativeLQR.initialize_states!(s, x̄)

# ## solve
@time IterativeLQR.solve!(s)

# ## solution
x_sol, u_sol = IterativeLQR.get_trajectory(s)
@show IterativeLQR.eval_obj(s.m_data.obj.costs, s.m_data.x, s.m_data.u, s.m_data.w)
@show s.s_data.iter[1]
# @show norm(goal(s.m_data.x[T], zeros(0), zeros(0)), Inf)

# ## visualize
vis= Visualizer()
# open(env.vis)
x_view = [[x_sol[1] for t = 1:15]..., x_sol..., [x_sol[end] for t = 1:15]...]
visualize(env, x_sol);

time = T*timestep
avg_velocity = -x_sol[end][1]/time*1000
power = sum([600000*transpose(u_sol[t])* u_sol[t] for t=1:T-1])
COT = power/Fg/-x_sol[end][1]
@show avg_velocity
@show COT

initialize_pleuro!(env.mechanism)
y = [copy(env.state)]
for i=1:3
    for t = 1:100
        if i==2
            neg = -1
        else
            neg = 1
        end
        step(env, env.state, neg*u_sol[t])
        push!(y, copy(env.state)) 
    end
end
visualize(env, y);

@save joinpath(@__DIR__, "results/long_pleuro_ilqr-"*Dates.format(now(), "yyyy-mm-dd HH:MM:SS")*".jld2") x_sol u_sol

name = "long_pleuro_ilqr-4-u.jld2"
@load joinpath(@__DIR__, "results/using/"*name) x_sol u_sol


searchdir(path,key) = filter(x->contains(x,key), readdir(path))
path = "echino-sim/trajopt/results/using/segments/"
key = ".jld2"
for name in searchdir(path,key)
    @load joinpath(@__DIR__, "results/using/segments/"*name) x_sol u_sol
    @show x_sol[end][1]
    file = matopen(path*"x_"*name[1:end-5]*".mat", "w")
    write(file, "x_sol", x_sol)
    close(file)
    file = matopen(path*"u_"*name[1:end-5]*".mat", "w")
    write(file, "u_sol", u_sol)
    close(file)
end

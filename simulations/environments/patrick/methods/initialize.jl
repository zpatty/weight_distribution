function get_patrick(; 
    timestep=0.05, 
    gravity=[0.0; 0.0; -9.81], 
    friction_coefficient=0.5,
    spring=0.0, 
    damper=1.0, 
    contact_feet=true, 
    contact_body=true,
    contact_type=:nonlinear,
    num_limbs=5,
    num_links_per_limb=2,
    mass_per_limb=0.036,
    mass_body=0.15,
    radius_b = 0.045,
    height_b=0.032,
    limb_length=0.13,
    mass_proximal=0.5,
    limits::Bool=true,
    joint_limits=[[-30.0, 30.0, -30.0, -70.0, -30.0, -70.0, -30.0, 30.0] * π / 180.0,
                  [ 30.0, 70.0,  30.0, -30.0,  30.0, -30.0,  30.0, 70.0] * π / 180.0],
    T=Float64)

    # define origin
    origin = Origin{T}(name=:origin)
    # define the two bodies which will form the tippe top once assembled
    factor = 1
    radius = height_b/2*factor
    radius_b = radius_b*factor
    height = limb_length/num_links_per_limb*factor
    height_b = height_b*factor
    mass = mass_per_limb/num_links_per_limb*factor
    mass_b = mass_body*factor
    α = 0.2

    arm_link = [[i+(j-1)*num_links_per_limb+1 for i=1:num_links_per_limb] for j=1:num_limbs]
    # masses = repeat([0.5*mass_per_limb],num_limbs*num_links_per_limb)
    masses = repeat([mass_proximal*mass_per_limb; (1-mass_proximal)*mass_per_limb],num_limbs)
    bodies = [Cylinder(radius_b, height_b, mass_b, name=:body, color=Dojo.cyan_light); [Cylinder(radius, height, masses[i], color=Dojo.cyan_light) for i = 1:num_links_per_limb*num_limbs]]
    # modify inertia of the first body to ensure we obtain the desired behavior
    #bodies[1].inertia = Diagonal([1.9, 2.1, 2.0])

    # define the joints, the 1st joint is between Origin and :sphere1
    # the 2nd joint is between :sphere1 and :sphere2
    
    #[[Symbol("joint1_", arm_link[j][1]); [Symbol("joint",i,"_",i+1) for i in arm_link[j][1:end-1]]] for j=1:num_limbs]
    if num_limbs == 3
        if3 = [0 0 1]
    else 
        if3 = zeros(num_limbs)
    end
    joints = [
        JointConstraint(Floating(origin, bodies[1]),
                name=:floating_joint);
            [[JointConstraint(Orbital(bodies[1], bodies[arm_link[j][1]],[0.0,0.0,1.0],
                axis_offset=svec_to_quat(Dojo.Lmat(Dojo.axis_angle_to_quaternion(π/2*[-sin(2*(j-1)*π/5); cos(2*(j-1)*π/5); 0.0]))*Dojo.vector(Dojo.RotZ(2*(j-1)*π/5))),# Dojo.quaternion_rotate(Dojo.axis_angle_to_quaternion(π/2*[-sin(2*(j-1)*π/5); cos(2*(j-1)*π/5); 0.0]),Dojo.RotZ(2*(j-1)*π/5)),
                parent_vertex=rot_mat_Z(2(j-1)π/5)*[-radius_b,0,0],
                child_vertex=[0,0,height/2]),
                name = Symbol("joint1_", arm_link[j][1])); 
            [JointConstraint(Orbital(bodies[i], bodies[i+1],[0.0,0.0,1.0],
                parent_vertex=[0,0,-height/2],
                child_vertex=[0,0,height/2]),
                name = Symbol("joint",i,"_",i+1)) for i in arm_link[j][1:end-1]]] for j=1:num_limbs]...
        ]
    # joints = [
    #     JointConstraint(Floating(origin, bodies[1]),
    #             name=:floating_joint);
    #         [[JointConstraint(Revolute(bodies[1], bodies[arm_link[j][1]],[1.0,0.0,0.0],
    #             axis_offset=Dojo.quaternion_rotate(Dojo.RotX(π/2),Dojo.RotZ(3π/2 - (5-2*(j-1+if3[j]))*π/5)),
    #             parent_vertex=rot_mat_Z(2(j-1+if3[j])π/5)*[-radius_b,0,0],
    #             child_vertex=[0,0,height/2]),
    #             name = Symbol("jointx1_", arm_link[j][1])); 
    #         JointConstraint(Revolute(bodies[1], bodies[arm_link[j][1]],[0.0,1.0,0.0],
    #             axis_offset=Dojo.quaternion_rotate(Dojo.RotX(π/2),Dojo.RotZ(3π/2 - (5-2*(j-1+if3[j]))*π/5)),
    #             parent_vertex=rot_mat_Z(2(j-1+if3[j])π/5)*[-radius_b,0,0],
    #             child_vertex=[0,0,height/2]),
    #             name = Symbol("jointy1_", arm_link[j][1]));
    #         [[JointConstraint(Revolute(bodies[i], bodies[i+1],[1.0,0.0,0.0],
    #             parent_vertex=[0,0,-height/2],
    #             child_vertex=[0,0,height/2]),
    #             name = Symbol("jointx",i,"_",i+1));
    #         JointConstraint(Revolute(bodies[i], bodies[i+1],[0.0,1.0,0.0],
    #             parent_vertex=[0,0,-height/2],
    #             child_vertex=[0,0,height/2]),
    #             name = Symbol("jointy",i,"_",i+1))] for i in arm_link[j][1:end-1]]...] for j=1:num_limbs]...
    #     ]

    mech = Mechanism(origin, bodies, joints, 
            gravity=gravity, 
            timestep=timestep, 
            spring=spring, 
            damper=damper)
    if limits
        for i=2:length(joints[2:end])
            joints[i] = Dojo.add_limits(mech, joints[i], 
                rot_limits=[SVector{2}(joint_limits[1][i-1],joint_limits[2][i-1]), SVector{2}(joint_limits[1][i-1],joint_limits[2][i-1])])
        end

        mech = Mechanism(origin, bodies, joints, 
            gravity=gravity, 
            timestep=timestep, 
            spring=spring, 
            damper=damper)
    end
        
#axis_offset=Dojo.quaternion_rotate(Dojo.RotY(π/2),Dojo.RotX(π/2)),
#Dojo.axis_angle_to_quaternion(π/2*rot_mat_Z(π/2-2π/5*2)*[-1,0,0])
    # joints = [
    #     JointConstraint(Floating(origin, bodies[1]),
    #             name=:floating_joint),
    #     JointConstraint(Orbital(bodies[1], bodies[2],[0.0,0.0,1.0],
    #             parent_vertex=[-radius_b,0,0],
    #             child_vertex=[0,0,height/2]),
    #             name = :joint1),

    #     JointConstraint(Orbital(bodies[2], bodies[3],[0.0,0.0,1.0],
    #             parent_vertex=[0,0,-height/2],
    #             child_vertex=[0,0,height/2]),
    #             name = :joint2),
    #     JointConstraint(Orbital(bodies[3], bodies[4],[0.0,0.0,1.0],
    #             parent_vertex=[0,0,-height/2],
    #             child_vertex=[0,0,height/2]),
    #             name = :joint3)]

    # define the two spherical contacts, each contact is attached to a body.
    contact_point = [0.0, 0.0, 0.0] # both contact spheres are located at the center of the bodies they are attached to.
    normal = [0.0, 0.0, 1.0] # the normal of the contact is always pointing in the upright direction because the floor is flat.
    # we specify the type of contact (impact, linear, nonlinear), the coefficient of friction, the radius of the contact sphere, etc.
    mesh = 15
    if contact_body
        normal_mesh = [[0.0; 0.0; 1.0] for i = 1:mesh]
        friction_coefficient_mesh = friction_coefficient * ones(mesh)
        mesh_circle = [[radius_b*cos(2*π/mesh*j),radius_b*sin(2*π/mesh*j),0.0] for j=1:mesh]

        contacts1 = [
            [contact_constraint(bodies[i[end]], normal,
                friction_coefficient=friction_coefficient,
                contact_origin=[0.0,0.0,height/2],
                contact_radius=radius,
                contact_type=contact_type) for i in arm_link[1:end]];
            [contact_constraint(body, normal,
                friction_coefficient=friction_coefficient,
                contact_origin=[0.0,0.0,-height/2],
                contact_radius=radius,
                contact_type=contact_type) for body in bodies[2:end]];
            contact_constraint(bodies[1], normal,
                friction_coefficient=friction_coefficient,
                contact_origin=contact_point, 
                contact_radius=height_b/2,
                contact_type=contact_type);
        ]

        # contacts1 = [
        #     contact_constraint(bodies[1], normal,
        #         friction_coefficient=friction_coefficient,
        #         contact_origin=contact_point, 
        #         contact_radius=height_b/2,
        #         contact_type=contact_type);
        #     [contact_constraint(bodies[arm_link[i][end]], normal,
        #         friction_coefficient=friction_coefficient,
        #         contact_origin=[0.0,0.0,height/2],
        #         contact_radius=radius,
        #         contact_type=contact_type) for i=1:length(arm_link)];
        #     [contact_constraint(body, normal,
        #         friction_coefficient=friction_coefficient,
        #         contact_origin=[0.0,0.0,-height/2],
        #         contact_radius=radius,
        #         contact_type=contact_type) for body in bodies[2:end]]
        # ]

        contacts2 = contact_constraint(fill(bodies[1],mesh), normal_mesh,
            friction_coefficient=friction_coefficient_mesh,
            contact_origins=mesh_circle,
            contact_radius=height_b/2*ones(mesh),
            contact_type=contact_type)
    
        mech = Mechanism(origin, bodies, joints, [contacts1;contacts2],
            gravity=gravity, 
            timestep=timestep, 
            spring=spring, 
            damper=damper)
    else
        mech = Mechanism(origin, bodies, joints,
            gravity=gravity, 
            timestep=timestep, 
            spring=spring, 
            damper=damper)
    end
    

    return mech
end

function rot_mat_Z(θ)
    mat = [cos(θ) -sin(θ) 0.0; sin(θ) cos(θ) 0.0; 0.0 0.0 1.0]
    return mat
end

function rot_2D_Z(θ)
    return [cos(θ) -sin(θ); sin(θ) cos(θ)]
end

function svec_to_quat(svec)
    return Quaternion(svec[1], svec[2],svec[3],svec[4])
end

function initialize_patrick!(mechanism::Mechanism; 
        body_position=[0.0; 0.0; 0.201], 
        body_orientation=[0.0; 0.0; 0.0 * π],
        ankle_orientation=0.0)

    #num_links_per_limb = trunc(Int,length(mechanism.bodies[2:end])/5)
    #arm_link = [[i+(j-1)*num_links_per_limb+1 for i=1:num_links_per_limb] for j=1:num_links]
    set_minimal_coordinates!(mechanism, get_joint(mechanism, :floating_joint), [body_position; body_orientation])
    # for j=1:5
    #     @show Symbol("joint", arm_link[j][1])
    #     set_minimal_coordinates!(mechanism, get_joint(mechanism, Symbol("joint", arm_link[j][1])), [0.0,0.0])
    #     for i=2:num_links
    #         @show arm_link[j][i]
    #         set_minimal_coordinates!(mechanism, get_joint(mechanism, Symbol("joint", arm_link[j][i])), [0.0,0.0])
    #     end
    # end

    
    for joint in mechanism.joints[2:end]
        set_minimal_coordinates!(mechanism, joint, [0.0,0.0])
    end
    # set_minimal_coordinates!(mechanism, get_joint(mechanism, Symbol("joint", 2)), [π/2,0.0])
    # set_minimal_coordinates!(mechanism, get_joint(mechanism, Symbol("joint", 3)), [π/2,0.0])

    zero_velocity!(mechanism)
end


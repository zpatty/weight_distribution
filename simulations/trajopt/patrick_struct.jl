
mutable struct Patrick
    env
    params

    function Patrick(params)
        mass_total = params[1]
        limb_length = params[2]
        mass_proximal = params[3]
        mass_in_limbs = params[4]
        
        patrick(
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
        obs = reset(environments[m]);
end
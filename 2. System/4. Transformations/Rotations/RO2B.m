function R_O2B = RO2B(rot_RPY)

    q_O2B = eul2quat(deg2rad(rot_RPY),"XYZ");
    R_O2B = quat2rotm(q_O2B);

end


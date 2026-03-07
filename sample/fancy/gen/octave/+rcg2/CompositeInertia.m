classdef CompositeInertia < handle
properties
    ip
    xm
    Ic_link1
    Ic_link2
    Ic_link3
    Ic_link4
    Ic_link5
    H
end

methods
    function obj = CompositeInertia(ip, xm)
    % Arguments:
    %  - thisclass.ctor.inertia» : a structure with the inertia properties
    %  - xm : the container of the coordinate transformation
    %    matrices for spatial motion vectors

        obj.ip = ip;
        obj.xm = xm;
        obj.Ic_link1 = zeros(6,6);
        obj.Ic_link2 = zeros(6,6);
        obj.Ic_link3 = zeros(6,6);
        obj.Ic_link4 = zeros(6,6);
        obj.Ic_link5 = ip.link5.tensor6D;
        obj.H = zeros(5,5);
    end

    function update_composite_inertia(obj)
    % Computes the spatial composite inertia of each link of the robot.
    % This method uses the current robot configuration. To use another one,
    % update first the coordinate transforms used by this instance.
    % The computed inertia are available as public members of this instance.

        % Contribution of link5 on link4
        obj.Ic_link4 = obj.ip.link4.tensor6D + obj.xm.link5_X_link4.mx' * obj.Ic_link5 * obj.xm.link5_X_link4.mx;

        % Contribution of link4 on link3
        obj.Ic_link3 = obj.ip.link3.tensor6D + obj.xm.link4_X_link3.mx' * obj.Ic_link4 * obj.xm.link4_X_link3.mx;

        % Contribution of link3 on link2
        obj.Ic_link2 = obj.ip.link2.tensor6D + obj.xm.link3_X_link2.mx' * obj.Ic_link3 * obj.xm.link3_X_link2.mx;

        % Contribution of link2 on link1
        obj.Ic_link1 = obj.ip.link1.tensor6D + obj.xm.link2_X_link1.mx' * obj.Ic_link2 * obj.xm.link2_X_link1.mx;


    end

    function update_JSIM(obj)
    % Computes the Joint Space Inertia Matrix of the robot Fancy.
    % This method uses the current composite inertia values.
    % See 'update_composite_inertia'.
    % The computed value is available in the public member 'H'
    % of this instance (and in 'F' for floating base models).

        F = obj.Ic_link5(:,3);
        obj.H(5, 5) = F(3);

        F = obj.xm.link5_X_link4.mx' * F;
        obj.H(5, 4) = obj.H(4, 5) = F(6);
        F = obj.xm.link4_X_link3.mx' * F;
        obj.H(5, 3) = obj.H(3, 5) = F(3);
        F = obj.xm.link3_X_link2.mx' * F;
        obj.H(5, 2) = obj.H(2, 5) = F(6);
        F = obj.xm.link2_X_link1.mx' * F;
        obj.H(5, 1) = obj.H(1, 5) = F(3);

        F = obj.Ic_link4(:,6);
        obj.H(4, 4) = F(6);

        F = obj.xm.link4_X_link3.mx' * F;
        obj.H(4, 3) = obj.H(3, 4) = F(3);
        F = obj.xm.link3_X_link2.mx' * F;
        obj.H(4, 2) = obj.H(2, 4) = F(6);
        F = obj.xm.link2_X_link1.mx' * F;
        obj.H(4, 1) = obj.H(1, 4) = F(3);

        F = obj.Ic_link3(:,3);
        obj.H(3, 3) = F(3);

        F = obj.xm.link3_X_link2.mx' * F;
        obj.H(3, 2) = obj.H(2, 3) = F(6);
        F = obj.xm.link2_X_link1.mx' * F;
        obj.H(3, 1) = obj.H(1, 3) = F(3);

        F = obj.Ic_link2(:,6);
        obj.H(2, 2) = F(6);

        F = obj.xm.link2_X_link1.mx' * F;
        obj.H(2, 1) = obj.H(1, 2) = F(3);

        F = obj.Ic_link1(:,3);
        obj.H(1, 1) = F(3);


    end


end % methods
end % class

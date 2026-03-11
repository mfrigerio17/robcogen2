classdef CompositeInertia < handle
properties
    ip
    xm
    Ic_trunk
    Ic_LF_hipassembly
    Ic_LF_upperleg
    Ic_LF_lowerleg
    Ic_RF_hipassembly
    Ic_RF_upperleg
    Ic_RF_lowerleg
    Ic_LH_hipassembly
    Ic_LH_upperleg
    Ic_LH_lowerleg
    Ic_RH_hipassembly
    Ic_RH_upperleg
    Ic_RH_lowerleg
    H
    F
end

methods
    function obj = CompositeInertia(ip, xm)
    % Arguments:
    %  - thisclass.ctor.inertia» : a structure with the inertia properties
    %  - xm : the container of the coordinate transformation
    %    matrices for spatial motion vectors

        obj.ip = ip;
        obj.xm = xm;
        obj.Ic_trunk = zeros(6,6);
        obj.Ic_LF_hipassembly = zeros(6,6);
        obj.Ic_LF_upperleg = zeros(6,6);
        obj.Ic_LF_lowerleg = ip.LF_lowerleg.tensor6D;
        obj.Ic_RF_hipassembly = zeros(6,6);
        obj.Ic_RF_upperleg = zeros(6,6);
        obj.Ic_RF_lowerleg = ip.RF_lowerleg.tensor6D;
        obj.Ic_LH_hipassembly = zeros(6,6);
        obj.Ic_LH_upperleg = zeros(6,6);
        obj.Ic_LH_lowerleg = ip.LH_lowerleg.tensor6D;
        obj.Ic_RH_hipassembly = zeros(6,6);
        obj.Ic_RH_upperleg = zeros(6,6);
        obj.Ic_RH_lowerleg = ip.RH_lowerleg.tensor6D;
        obj.H = zeros(12,12);
        obj.F = zeros(6,12);
    end

    function update_composite_inertia(obj)
    % Computes the spatial composite inertia of each link of the robot.
    % This method uses the current robot configuration. To use another one,
    % update first the coordinate transforms used by this instance.
    % The computed inertia are available as public members of this instance.

        % Contribution of RH_lowerleg on RH_upperleg
        obj.Ic_RH_upperleg = obj.ip.RH_upperleg.tensor6D + obj.xm.RH_lowerleg_X_RH_upperleg.mx' * obj.Ic_RH_lowerleg * obj.xm.RH_lowerleg_X_RH_upperleg.mx;

        % Contribution of RH_upperleg on RH_hipassembly
        obj.Ic_RH_hipassembly = obj.ip.RH_hipassembly.tensor6D + obj.xm.RH_upperleg_X_RH_hipassembly.mx' * obj.Ic_RH_upperleg * obj.xm.RH_upperleg_X_RH_hipassembly.mx;

        % Contribution of RH_hipassembly on trunk
        obj.Ic_trunk = obj.ip.trunk.tensor6D + obj.xm.RH_hipassembly_X_trunk.mx' * obj.Ic_RH_hipassembly * obj.xm.RH_hipassembly_X_trunk.mx;

        % Contribution of LH_lowerleg on LH_upperleg
        obj.Ic_LH_upperleg = obj.ip.LH_upperleg.tensor6D + obj.xm.LH_lowerleg_X_LH_upperleg.mx' * obj.Ic_LH_lowerleg * obj.xm.LH_lowerleg_X_LH_upperleg.mx;

        % Contribution of LH_upperleg on LH_hipassembly
        obj.Ic_LH_hipassembly = obj.ip.LH_hipassembly.tensor6D + obj.xm.LH_upperleg_X_LH_hipassembly.mx' * obj.Ic_LH_upperleg * obj.xm.LH_upperleg_X_LH_hipassembly.mx;

        % Contribution of LH_hipassembly on trunk
        obj.Ic_trunk = obj.Ic_trunk + obj.xm.LH_hipassembly_X_trunk.mx' * obj.Ic_LH_hipassembly * obj.xm.LH_hipassembly_X_trunk.mx;

        % Contribution of RF_lowerleg on RF_upperleg
        obj.Ic_RF_upperleg = obj.ip.RF_upperleg.tensor6D + obj.xm.RF_lowerleg_X_RF_upperleg.mx' * obj.Ic_RF_lowerleg * obj.xm.RF_lowerleg_X_RF_upperleg.mx;

        % Contribution of RF_upperleg on RF_hipassembly
        obj.Ic_RF_hipassembly = obj.ip.RF_hipassembly.tensor6D + obj.xm.RF_upperleg_X_RF_hipassembly.mx' * obj.Ic_RF_upperleg * obj.xm.RF_upperleg_X_RF_hipassembly.mx;

        % Contribution of RF_hipassembly on trunk
        obj.Ic_trunk = obj.Ic_trunk + obj.xm.RF_hipassembly_X_trunk.mx' * obj.Ic_RF_hipassembly * obj.xm.RF_hipassembly_X_trunk.mx;

        % Contribution of LF_lowerleg on LF_upperleg
        obj.Ic_LF_upperleg = obj.ip.LF_upperleg.tensor6D + obj.xm.LF_lowerleg_X_LF_upperleg.mx' * obj.Ic_LF_lowerleg * obj.xm.LF_lowerleg_X_LF_upperleg.mx;

        % Contribution of LF_upperleg on LF_hipassembly
        obj.Ic_LF_hipassembly = obj.ip.LF_hipassembly.tensor6D + obj.xm.LF_upperleg_X_LF_hipassembly.mx' * obj.Ic_LF_upperleg * obj.xm.LF_upperleg_X_LF_hipassembly.mx;

        % Contribution of LF_hipassembly on trunk
        obj.Ic_trunk = obj.Ic_trunk + obj.xm.LF_hipassembly_X_trunk.mx' * obj.Ic_LF_hipassembly * obj.xm.LF_hipassembly_X_trunk.mx;

    end

    function update_JSIM(obj)
    % Computes the Joint Space Inertia Matrix of the robot HyQ.
    % This method uses the current composite inertia values.
    % See 'update_composite_inertia'.
    % The computed value is available in the public member 'H'
    % of this instance (and in 'F' for floating base models).

        F = obj.Ic_RH_lowerleg(:,3);
        obj.H(12, 12) = F(3);

        F = obj.xm.RH_lowerleg_X_RH_upperleg.mx' * F;
        obj.H(12, 11) = obj.H(11, 12) = F(3);
        F = obj.xm.RH_upperleg_X_RH_hipassembly.mx' * F;
        obj.H(12, 10) = obj.H(10, 12) = F(3);
        obj.F(:,12) = obj.xm.RH_hipassembly_X_trunk.mx' * F;

        F = obj.Ic_RH_upperleg(:,3);
        obj.H(11, 11) = F(3);

        F = obj.xm.RH_upperleg_X_RH_hipassembly.mx' * F;
        obj.H(11, 10) = obj.H(10, 11) = F(3);
        obj.F(:,11) = obj.xm.RH_hipassembly_X_trunk.mx' * F;

        F = obj.Ic_RH_hipassembly(:,3);
        obj.H(10, 10) = F(3);

        obj.F(:,10) = obj.xm.RH_hipassembly_X_trunk.mx' * F;

        F = obj.Ic_LH_lowerleg(:,3);
        obj.H(9, 9) = F(3);

        F = obj.xm.LH_lowerleg_X_LH_upperleg.mx' * F;
        obj.H(9, 8) = obj.H(8, 9) = F(3);
        F = obj.xm.LH_upperleg_X_LH_hipassembly.mx' * F;
        obj.H(9, 7) = obj.H(7, 9) = F(3);
        obj.F(:,9) = obj.xm.LH_hipassembly_X_trunk.mx' * F;

        F = obj.Ic_LH_upperleg(:,3);
        obj.H(8, 8) = F(3);

        F = obj.xm.LH_upperleg_X_LH_hipassembly.mx' * F;
        obj.H(8, 7) = obj.H(7, 8) = F(3);
        obj.F(:,8) = obj.xm.LH_hipassembly_X_trunk.mx' * F;

        F = obj.Ic_LH_hipassembly(:,3);
        obj.H(7, 7) = F(3);

        obj.F(:,7) = obj.xm.LH_hipassembly_X_trunk.mx' * F;

        F = obj.Ic_RF_lowerleg(:,3);
        obj.H(6, 6) = F(3);

        F = obj.xm.RF_lowerleg_X_RF_upperleg.mx' * F;
        obj.H(6, 5) = obj.H(5, 6) = F(3);
        F = obj.xm.RF_upperleg_X_RF_hipassembly.mx' * F;
        obj.H(6, 4) = obj.H(4, 6) = F(3);
        obj.F(:,6) = obj.xm.RF_hipassembly_X_trunk.mx' * F;

        F = obj.Ic_RF_upperleg(:,3);
        obj.H(5, 5) = F(3);

        F = obj.xm.RF_upperleg_X_RF_hipassembly.mx' * F;
        obj.H(5, 4) = obj.H(4, 5) = F(3);
        obj.F(:,5) = obj.xm.RF_hipassembly_X_trunk.mx' * F;

        F = obj.Ic_RF_hipassembly(:,3);
        obj.H(4, 4) = F(3);

        obj.F(:,4) = obj.xm.RF_hipassembly_X_trunk.mx' * F;

        F = obj.Ic_LF_lowerleg(:,3);
        obj.H(3, 3) = F(3);

        F = obj.xm.LF_lowerleg_X_LF_upperleg.mx' * F;
        obj.H(3, 2) = obj.H(2, 3) = F(3);
        F = obj.xm.LF_upperleg_X_LF_hipassembly.mx' * F;
        obj.H(3, 1) = obj.H(1, 3) = F(3);
        obj.F(:,3) = obj.xm.LF_hipassembly_X_trunk.mx' * F;

        F = obj.Ic_LF_upperleg(:,3);
        obj.H(2, 2) = F(3);

        F = obj.xm.LF_upperleg_X_LF_hipassembly.mx' * F;
        obj.H(2, 1) = obj.H(1, 2) = F(3);
        obj.F(:,2) = obj.xm.LF_hipassembly_X_trunk.mx' * F;

        F = obj.Ic_LF_hipassembly(:,3);
        obj.H(1, 1) = F(3);

        obj.F(:,1) = obj.xm.LF_hipassembly_X_trunk.mx' * F;

    end

    function ic = base_Ic(obj)
        ic = obj.Ic_trunk;
    end

end % methods
end % class

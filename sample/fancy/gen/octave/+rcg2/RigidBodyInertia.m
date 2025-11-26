classdef RigidBodyInertia < handle
    properties
        com_
        tensor6D
    end
    methods
        function m = mass(obj)
            m = obj.tensor6D(6,6);
        end

        function c = com(obj)
            c = obj.com_;
        end

        function I = rotationalInertia(obj)
            I = obj.tensor6D(1:3,1:3);
        end

        function obj = RigidBodyInertia(mass, com, tensor)
            obj.com_     = com;
            obj.tensor6D = obj.makeTensor6D(mass, com, tensor);
        end

        function changeMass(obj, newmass)
            % scale the tensor
            obj.tensor6D = obj.tensor6D * newmass / obj.mass;
        end

        function changeCoM(obj, newcom)
            obj.com_ = newcom;
            m = obj.mass();
            aux = newcom * m;
            newblock = [  0,    -aux(3),  aux(2);
                         aux(3),  0,      -aux(1);
                        -aux(2),  aux(1),  0 ];
            oldblock = obj.tensor6D(1:3,4:6);
            obj.tensor6D(1:3,1:3) = obj.tensor6D(1:3,1:3) + ...
                    (newblock*newblock' - oldblock * oldblock') / m;
            obj.tensor6D(1:3,4:6) = newblock;
            obj.tensor6D(4:6,1:3) = newblock';
        end

        function changeRotationalInertia(obj, tensor3D)
            obj.tensor6D(1:3,1:3) = tensor3D;
        end
    end

    methods(Static)
        function I = makeTensor3D(ixx, iyy, izz, ixy, ixz, iyz)
            I = [[ixx, -ixy, -ixz];[-ixy, iyy, -iyz];[-ixz, -iyz, izz]];
        end

        function I = makeTensor6D(mass, com, tensor3D)
            block = [  0,    -com(3),  com(2);
                     com(3),  0,      -com(1);
                    -com(2),  com(1),  0 ] * mass;
            I = [tensor3D, block; block', mass*eye(3)];
        end
    end

end

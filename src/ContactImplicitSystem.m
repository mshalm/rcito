classdef ContactImplicitSystem < DiscreteDynamicalSystem
    %CONTACTIMPLICITSYSTEM Class container for dynamics of rigid body
    %system that undergoes contact.
    
    properties
        H
        G
        C
        B
        J
        phi
        dG
        dCv
        nq
        nv
        suffix
        cleanup
    end
    
    methods
        function obj = ContactImplicitSystem(dynamicsGenerator, suffix)
            [q, v, u, T, V, B, phi] = dynamicsGenerator();
            
            obj = ...
                obj@DiscreteDynamicalSystem(numel(q) + numel(v), numel(u));
            obj.nq = numel(q);
            obj.nv = numel(v);
            
            if nargin < 2
                suffix = '';
            end
            suffix = generateContactDynamics(q, v, T, V, B, phi, suffix);
            obj.suffix = suffix;
            
            obj.H = eval(['@H',suffix]);
            obj.G = eval(['@G',suffix]);
            obj.C = eval(['@C',suffix]);
            obj.B = eval(['B',suffix]);
            obj.J = eval(['@J',suffix]);
            obj.phi = eval(['@phi',suffix]);
            obj.dG = eval(['@dG',suffix]);
            obj.dCv = eval(['@dCv',suffix]);
            
            obj.cleanup = onCleanup(@() obj.delete());
        end
        
        function [xp] = dynamics(obj, xm, u, h)
            t = 0;
            q_cur = xm(1:obj.nq);
            v_cur = xm((obj.nq + 1):end);
            while (t < h)
                h_try = h - t;
                phi_cur = obj.phi(q_cur);
                active_cur = obj.activeContacts(q_cur);
                
                [q_try, v_try] = obj.step(q_cur, v_cur, u, h_try);
                phi_try = obj.phi(q_try);
                active_try = obj.activeContacts(q_cur);
                new_contacts = (active_try - active_cur) > 0;
                if (any(new_contacts))
                    % new contact initiated
                    % approximate phi as linear in time, and find
                    % minimum h for which there is a zero crossing
                    % on an index in new_contacts
                    % TODO: replace linear approximation with Hermitian
                    % spline
                    phi0 = phi_cur;
                    dphi0 = 0.5*(obj.J(phi_cur)*v_cur + ...
                        obj.J(phi_try)*v_try);
                    phi0 = phi0(new_contacts);
                    dphi0 = dphi0(new_contacts);
                    h_min = min(-phi0 ./ dphi0);
                    % make smaller step to get state right before impact
                    [q_cur, v_cur] = obj.step(q_cur, v_cur, u, h_min);
                    
                    % get state post impact
                    [q_cur, v_cur] = obj.step(q_cur, v_cur, u, 0);
                    t = t + h_min;
                else
                   t = t + h_try;
                   q_cur = q_try;
                   v_cur = v_try;
                   break;
                end
            end
            xp = [q_cur; v_cur];
        end
        
        function vect = activeContacts(obj, q)
            e = 1e-4;
            vect = logical(obj.phi(q) < e);
        end
        
        function [qp, vp] = step(obj, qm, vm, u, h)
            xm = [qm; vm];
            Hm = obj.H(qm);
            Gm = obj.G(qm);
            Cm = obj.C(xm);
            Bm = obj.B;
            Jm = obj.J(qm);
            active = obj.activeContacts(qm);
            
            dGm = obj.dG(qm);
            dCvm = obj.dCv(xm);
            dCvm_dq = dCvm(:,1:obj.nq);
            dCvm_dv = dCvm(:,(obj.nq + 1):end);
            
            % limit contacts to those that are active
            Jm = Jm(active, :);
            
            k = Bm*u - Cm*vm - Gm;
            dk_dq = -dCvm_dq - dGm;
            dk_dv = -dCvm_dv;
            
            H_hat = Hm - h^2 * dk_dq - h * dk_dv;
            k_hat = k - dk_dv * vm;
            
            r = Hm * vm + h * k_hat;
            if (nnz(active) > 0)
                M = Jm * (H_hat \ Jm');
                l = Jm * (H_hat \ r);
                [~, lambda, retcode] = LCPSolve(M,l);

                vp = H_hat \ (r + Jm' * lambda);
            else
                vp = H_hat \ r;
            end
            qp = qm + h*vp;
        end
        
        function delete(obj)
            delete(['H',obj.suffix]);
            delete(['G',obj.suffix]);
            delete(['C',obj.suffix]);
            delete(['B',obj.suffix]);
            delete(['J',obj.suffix]);
            delete(['phi',obj.suffix]);
            delete(['dG',obj.suffix]);
            delete(['dCv',obj.suffix]);
        end
    end
    
end


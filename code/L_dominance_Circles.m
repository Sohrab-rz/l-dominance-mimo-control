function []=L_dominance_Circles(Sigma,q_ii)
    Sigma=tf2sym(Sigma);
    G=tf2sym(q_ii);
   
    w=logspace(-5,6,100); 
    q=0:(pi/50):(2*pi);
    figure
    nyquist(q_ii);
    
    R=zeros(1,length(w));
    
    hold on
    C=subs(G,complex(0,w));                     % Center of Circles
    R=R+abs(subs(Sigma,complex(0,w)));        % Radius Circles
    for k=1:length(C)
        plot((R(k)*cos(q))+real(C(k)),(R(k)*sin(q))+imag(C(k)),'r-.') 
    end    
    grid on
    hold off
 
end



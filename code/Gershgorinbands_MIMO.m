function[]=Gershgorinbands_MIMO(G)
switch class(G)
    case 'tf'                                                              % Transfer Function Form
        G_TF=G; [num,den]=tfdata(G);  
        syms s
         for i=1:size(num,1)                                                                    
            for j=1:size(num,2)                                                                 
                num_symbolic(i,j)=poly2sym(num{i,j}); den_symbolic(i,j)=poly2sym(den{i,j}); 
            end
        end
        G_S=simplify(num_symbolic./den_symbolic);                            % Converting to Symbolic Form
    case 'sym'                                                             % Symbolic Form
        G_S=simplify(G); [a,b]=numden(G_S);
        for i=1:size(a,1)                                                                      
            for j=1:size(a,2)                                                                  
                num{i,j}=sym2poly(a(i,j)); den{i,j}=sym2poly(b(i,j));
                G_TF(i,j)=tf( num{i,j}, den{i,j})                          % Converting to Transfer Function Form
            end
        end
end
g=G_TF;

%%                                                                  Greshgorin Bands
[n,m]=size(g);                                                             % Number of Inputs and Outputs
w=logspace(-5,6,150);                                                      % Frequency Range
q=0:(pi/100):(2*pi);
figure
for i=1:n
    for j=1:m
        if i==j
            subplot(n,m,(i-1)*size(num,2)+j); nyquist(g(i,i));             % Nyquist Diagram of G(i,i)
            title(['Nyquist Diagram of G(',num2str(i),',',num2str(j),')'])
%%                                                                Row Diagonal Dominant
            for iest=i:i                                                                                   
                R=zeros(1,length(w));
                for jest=1:m
                    if iest~=jest
                        hold on
                        C=subs(G_S(i,j),complex(0,w));                     % Center of Circles
                        R=R+abs(subs(G_S(iest,jest),complex(0,w)));        % Radius Circles
                        if jest==m
                            for k=1:length(C)
                                plot((R(k)*cos(q))+real(C(k)),(R(k)*sin(q))+imag(C(k)),'r-') 
                            end
                        end
                        if (iest==m) && (jest==m-1)
                            for k=1:length(C)
                                plot((R(k)*cos(q))+real(C(k)),(R(k)*sin(q))+imag(C(k)),'r-')  
                            end
                        end
                        hold off
                    end
                end
            end
        else
            subplot(n,m,(i-1)*size(num,2)+j); nyquist(g(i,j));             % Nyquist Diagram of G(i,j)
            title(['Nyquist Diagram of G(',num2str(i),',',num2str(j),')'])
        end
    end
end
%%                                                               The End of Program.


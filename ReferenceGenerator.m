function [Xref,Yref,Psiref] = ReferenceGenerator(type,Ts_ref,N,scale)

    switch type
        case 'sharp_turn'
            t = (0:(N-1))*Ts_ref;
            xref = 7*t;
            yref = 15*[0*(1:300) 0.01*(1:800) 8*ones(1,1000)];
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];

        case 'line'
            t = (0:(N-1))*Ts_ref;
            xref = t;
            yref = 0*ones(1,N);
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];

        case 'smooth_curve'
            t = (0:(N-1))*Ts_ref;
            xref = (300-t).*sin(0.15*t);
            yref= -(300-t).*cos(0.15*t)+300;
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];

        case 'circle'
            t = (0:(N-1))*Ts_ref;
            xref = 30*sin(0.15*t);
            yref= -30*cos(0.15*t)+30;
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];

        case 'infinite'
            t = (0:(N-1))*Ts_ref;
            xref = scale*cos(t);
            yref = scale*sin(2*t) / 2;
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];

        case 'ascent_sin'
            t = (0:(N-1))*Ts_ref;
            xref = 1.1*t;
            yref = 8*sin(0.02*t+0.0004*t.*t);
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];
    end
    
    Xref=xref';%change row into column
    Yref=yref';%change row into column
    Psiref=psiref';%change row into column
end
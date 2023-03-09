function [Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale)
% look at choice t, define N, ref_dis
    switch type
        case 'sharp_turn'
            t = (0:(N-1))*ref_dis;
            xref = 7*t;
            yref = 5*[0*(1:300) 0.01*(1:800) 8*ones(1,1000)];
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];

        case 'line'
            t = (0:(N-1))*ref_dis;
            xref = t;
            yref = 0*ones(1,N);
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];

        case 'smooth_curve'
            t = (0:(N-1))*ref_dis;
            xref = (300-t).*sin(0.15*t);
            yref= -(300-t).*cos(0.15*t)+300;
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];

        case 'circle'
            t = (0:(N-1))*ref_dis;
            xref = scale*sin(0.15*t);
            yref= -scale*cos(0.15*t)+30;
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];

        case 'infinite'
%           t = [(0:(N-1))*ref_dis];
            t = [(0:(N-1))*ref_dis; (0:(N-1))*2*ref_dis];
            xref = scale*cos(t);
            yref = scale*sin(2*t) / 2;
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];

        case 'ascent_sin'
            t = (0:(N-1))*ref_dis;
            xref = 1.1*t;
            yref = 8*sin(0.02*t+0.0004*t.*t);
            psiref=atan2(yref(2:N)-yref(1:N-1),xref(2:N)-xref(1:N-1)); 
            psiref=[psiref(1) psiref];
    end
    
    Xref=xref';%change row into column
    Yref=yref';%change row into column
    Psiref=psiref';%change row into column
end
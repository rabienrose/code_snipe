warning('off');
global ps us cof;
clear
a=1;
v0=1;
dt=0.1;
p_true=[];
h=5;
u_max=1;
u_min=0.1;
f=0.1;
noise=0.1;
rr=0.00004;
for i=1:100
   p_true(i)=v0*dt*i+(i*dt)^2*a*0.5; 
end

pf=0:0.1:50;
for i=1:length(pf)
    track_table(i).frameid=[];
    track_table(i).u=[];
    track_table(i).lost=false;
end

for i=1:length(pf)
    for j=1:length(p_true)
        d = pf(i)-p_true(j);
        if d>0
            u = f*h/d;
            if u<u_max && u>u_min
                track_table(i).frameid(end+1)=j;
                track_table(i).u(end+1)=u;
            end
        end
    end
end

state.p=0;
state.v=v0;
state.slide=[];
state.cov=zeros(2);
hold on;
for i=1:100
    cur_frame=i;
    n=size(state.cov,1);
    j=[eye(2) zeros(2,n-2)
        1 0 zeros(1,n-2)
        zeros(n-2,2),eye(n-2)];
    state.cov = j*state.cov*j';
    n=size(state.cov,1);
    slide_len= length(state.slide);
    state.slide(2:slide_len+1) = state.slide(1:slide_len);
    state.slide(1).p=state.p;
    state.slide(1).frame_id=cur_frame-1;
    state.p=state.p+dt*state.v+0.5*dt^2*a;
    state.v=state.v+a*dt;
    phi=[1 dt
         0 1];
    Q=[0 0
       0 noise*dt];
    Q = blkdiag( Q, zeros(n-2,n-2) );
    ErrCov11 = ( state.cov(1:2,1:2) + ( state.cov(1:2,1:2) )' )/2;
    ErrCov12 = ( state.cov(1:2,3:n) + ( state.cov(3:n,1:2) )' )/2;
    ErrCov22 = ( state.cov(3:n,3:n) + ( state.cov(3:n,3:n) )' )/2;

    crosscov = phi*ErrCov12;
    state.cov = [phi*ErrCov11*phi' crosscov;
                crosscov' ErrCov22] + Q;
    
    frame_ids=[];
    for j=1:length(state.slide)
        frame_ids(end+1)=state.slide(j).frame_id;
    end
    r=[];
    H=[];
    for j=1:length(track_table)
        if length(track_table(j).frameid)<4
            track_table(j).lost=true;
            continue;
        end
        if track_table(j).lost==true
            continue;
        end
        if track_table(j).frameid(end)<cur_frame
            track_table(j).lost=true;
            [C slide_mask track_mask] = intersect(frame_ids,track_table(j).frameid);
            pf_ts=[];
            ps=[];
            us=[];
            for k=1:length(track_mask)
                p_t=state.slide(slide_mask(k)).p;
                u=track_table(j).u(track_mask(k));
                pf_t=f*h/u+p_t;
                pf_ts(end+1)=pf_t;
                ps(end+1)=p_t;
                us(end+1)=u;
            end
            pf_cur=mean(pf_ts);
            cof=h*f;
            fun=@cost;
            %[x,fval,exitflag,output,grad,hessian]=fminunc(fun, pf_cur);
            %pf_cur=x;
            r_j=[];
            Hx_j=[];
            Hf_j=[];
            for k=1:length(track_mask)
                p_t=state.slide(slide_mask(k)).p;
                u=track_table(j).u(track_mask(k));
                u_obs=f*h/(pf_cur-p_t);
                r_k=u_obs-u;
                r_j(end+1,1)=r_k;
                Hx_k = zeros(1,size(state.cov,1));
                Hx_k(1, 2+slide_mask(k)) = f*h/(pf_cur-p_t)^2;
                Hx_j(end+1,:)=Hx_k;
                Hf_k=-f*h/(pf_cur-p_t)^2;
                Hf_j(end+1,1)=Hf_k;
            end
            leftNullHf = null(Hf_j');
            if ~isempty(leftNullHf)
                r_j = leftNullHf'*r_j;
                Hx_j  = leftNullHf'*Hx_j;
            end
            r(end+1:end+length(r_j),1)=r_j;
            H(end+1:end+size(Hx_j,1),:)=Hx_j;
        end
    end
    if isempty(r)
        continue;
    end
    RQ=eye(length(r))*rr;
    sig = H*state.cov*H' + RQ;
    if cond( sig ) < 1e10
        K  = state.cov*H' / sig;        
        dX = K*r;
        dX = dX(:)';                
        state.cov = ( eye(size(state.cov,1))-K*H )*state.cov;
        if cur_frame==50
        end
        state.p = state.p+dX(1);
        state.v = state.v + dX(2); 
        for k = 1:length(state.slide)
            state.slide(k).p = state.slide(k).p+dX(2+k);
        end
    end
    for n=length(state.slide):-1:1
        slide_empty=true;
        for j=1:length(track_table)
            if track_table(j).lost==false && ~isempty(find(state.slide(n).frame_id==track_table(j).frameid)) 
                slide_empty= false;
                break;
            end
        end
        if slide_empty==false
            break;
        end
        state.slide(n)=[];
        state.cov(n+2,:)=[];
        state.cov(:,n+2)=[];
    end
    
    plot(cur_frame,state.cov(2,2),'r*');
end


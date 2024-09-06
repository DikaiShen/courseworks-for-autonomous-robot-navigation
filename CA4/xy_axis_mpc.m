function j = xy_axis_mpc(K,dt,p_0,v_0,a_0,pt,vt,at)
 %Implement your code here

 %Dikai Shen
 %A0285139W

 w1 = 100;
 w2 = 1;
 w3 = 1;
 w4 = 1;
 w5 = 1e4;
 %% construct the prediction matrix

 [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K,dt,p_0,v_0,a_0);
 %% construct and solve the optimizaton process

 BpNew = Bp - pt;
 BvNew = Bv - vt;
 BaNew = Ba - at;


 %apply the soft constraints
 H = blkdiag(w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta),w5*eye(K));
 F = [w1*BpNew'*Tp+w2*BvNew'*Tv+w3*BaNew'*Ta zeros(1,K)];
 
 A = [Tv eye(K);-Tv -eye(K);Ta eye(K);-Ta -eye(K); eye(K) zeros(K);-eye(K) zeros(K);zeros(size(Ta)) -eye(K)];
 b = [6*ones(20,1)-Bv;6*ones(20,1)+Bv;3*ones(20,1)-Ba;3*ones(20,1)+Ba; 3*ones(20,1); 3*ones(20,1);zeros(K,1)];
 %apply soft constraints on velocity and acceleration, and hard constraint
 %on the jerk

 %jerk result
 J = quadprog(H,F,A,b);

 j = J(1);
end
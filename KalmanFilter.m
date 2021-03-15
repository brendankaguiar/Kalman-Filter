%Code written and edited by Brendan Aguiar
function  [s] = KalmanFilter(s)
Xk = s.A * s.x;
Pk = s.A * s.P * transpose(s.A) + s.Q;%a priori prediction covariance
Kk = Pk * s.H / (s.H * Pk * transpose(s.H) + s.R);
s.x = Xk + Kk * (s.z - (s.H * Xk));
s.P = Pk - (Kk * s.H * Pk);
end

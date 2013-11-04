theta_diff1 = t1_next_state(5,:)-t1_next_state(6,:);
theta_diff2 = t2_next_state(5,:)-t2_next_state(6,:);
theta_diff3 = t3_next_state(5,:)-t3_next_state(6,:);

[z,ans] = p_win(theta_diff3',t2_new_weights,100);
p_x=ans;
p_x=p_x./sum(p_x);
log_p_x=log(p_x);
-log_p_x*p_x'
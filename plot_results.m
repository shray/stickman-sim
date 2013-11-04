figure(50)

subplot(4,1,1)
plot(ground_truth,'k')
hold on
plot(t1_estimates,'r')

subplot(4,1,2)
plot(ground_truth,'k')
hold on
plot(t2_estimates,'g')

subplot(4,1,3)
plot(ground_truth,'k')
hold on
plot(t3_estimates,'b')

subplot(4,1,4)
plot(abs(t1_estimates-ground_truth), 'r')
hold on
plot(abs(t2_estimates-ground_truth), 'g')
plot(abs(t3_estimates-ground_truth), 'b')




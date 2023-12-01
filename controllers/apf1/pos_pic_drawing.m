load position_data1
a1 = pic_posi;
load position_data2
a2 = pic_posi;
load position_data3
a3 = pic_posi;
load target_data
b = pic_target;
figure(1)
hold on
plot(a1(:,1),a1(:,2),'-g')
plot(a2(:,1),a2(:,2),'--b')
plot(a3(:,1),a3(:,2),'-.r')
plot(b(:,1),b(:,2),'-k','linewidth',2)
legend('进攻方智能体1','进攻方智能体2','进攻方智能体3','逃逸方智能体')
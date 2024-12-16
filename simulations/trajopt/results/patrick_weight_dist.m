
%%
clear
close all
filenames = {'distal','brittlestar', 'robot', 'central'};
weight_pct = {'98%','75%','55%','14%'}; 
%filenames = {'brittlestar','central'};
centralopt = 'centralopt_';
centralopt ='n_';
hold on
contact_inds = [9,15];
tvec = 0:0.05:10;
for j=1:length(filenames)
    load(strcat('patrick_contacts_',centralopt,filenames{j},'.mat'))
    load(strcat('patrick_maximal_state_',centralopt,filenames{j},'.mat'))
    for i=1:length(contact_inds)
        contact = contacts{contact_inds(i)};
        num_zero{j,i} = sum(contact<0.0001);
%         stairs(contact(2:end))
%         legend_str{i} = num2str(bodies(i));
    end
    num_zero{j,i+1} = sum(sum([contacts{16:31}]<0.0001));
    body_5_mat = [state{5}{:}];
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_5_mat(1,:)+body_1_mat(1,:))*100,'Linewidth',1.3)
    % plot(tvec,contact)
    legend_str{j} = filenames{j};
    distance(j) = -body_1_mat(1,end)+body_1_mat(1,1);
end
num_zero
distance
legend(legend_str)
xlim([0 10])
xlabel('Time (s)')
ylabel('Front arm x location relative to body (cm)')

figure 
hold on
for j=1:length(filenames)
    load(strcat('patrick_maximal_state_',centralopt,filenames{j},'.mat'))
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'Linewidth',1.3)
    legend_str{j} = weight_pct{j};
end
cm = lines(7);
for j=1:length(filenames)
    load(strcat('patrick_maximal_state_h_',filenames{j},'.mat'))
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'--','Linewidth',1.3,'color',cm(j,:))
    %legend_str{j} = weight_pct{j};
end
leg = legend(legend_str);
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Distance (cm)')
ylim([-1 170])
set(gca,'FontSize',14)

input.data = cell2mat(num_zero);
latexTable(input)

figure
hold on
files = [1, 4];
cm = lines(7);
for k=1:2
    j = files(k);
    load(strcat('patrick_contacts_',centralopt,filenames{j},'.mat'))
    load(strcat('patrick_maximal_state_',centralopt,filenames{j},'.mat'))
    contact = contacts{contact_inds(1)};
    legend_str{k} = weight_pct{j};
     plot(tvec(1:61),contact(1:61),'Linewidth',1.3,'color',cm(j,:))
end
leg = legend(legend_str);
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Leg tip vertical distance (cm)')
set(gca,'FontSize',14)

%%
tvec = 0:0.05:2.5;
figure 
hold on
for j=1:length(filenames)
    load(strcat('patrick_maximal_state_12_',filenames{j},'.mat'))
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'Linewidth',1.3)
    legend_str{j} = weight_pct{j};
end
load(strcat('patrick_maximal_state_ref.mat'))
body_1_mat = [state{1}{:}];
plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'--','Linewidth',1.3)
legend_str{j+1} = 'ref';
leg = legend(legend_str);

title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Distance (cm)')
set(gca,'FontSize',14)

figure
hold on
files = [1, 4];
cm = lines(7);
for j=1:length(filenames)
    load(strcat('patrick_contacts_12_',filenames{j},'.mat'))
    load(strcat('patrick_maximal_state_12_',filenames{j},'.mat'))
    for i=1:length(contact_inds)
        contact = contacts{contact_inds(i)};
        num_zero{j,i} = sum(contact<0.0001);
%         stairs(contact(2:end))
%         legend_str{i} = num2str(bodies(i));
    end
    num_zero{j,i+1} = sum(sum([contacts{16:31}]<0.0001));
    body_5_mat = [state{5}{:}];
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_5_mat(1,:)+body_1_mat(1,:))*100,'Linewidth',1.3)
    legend_str{j} = weight_pct{j};
    distance(j) = -body_1_mat(1,end)+body_1_mat(1,1);
end
num_zero
distance
legend(legend_str)
xlabel('Time (s)')
ylabel('Front arm x location relative to body (cm)')

figure
hold on
files = [1, 4];
cm = lines(7);
c_i = 15;
for k=1:2
    j = files(k);
    load(strcat('patrick_contacts_12_',filenames{j},'.mat'))
    load(strcat('patrick_maximal_state_12_',filenames{j},'.mat'))
    contact = contacts{c_i};
    legend_str{k} = weight_pct{j};
    plot(tvec,contact,'Linewidth',1.3,'color',cm(j,:))
end
load(strcat('patrick_contacts_ref.mat'))
contact = contacts{c_i};
plot(tvec,contact,'--','Linewidth',1.3,'color',cm(5,:))
legend_str{k+1} = 'ref';
leg = legend(legend_str);
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Leg tip vertical distance (cm)')
set(gca,'FontSize',14)


load(strcat('patrick_actuators_ref.mat'))
input_ref = reshape(cell2mat(inputs),20,50);
load('patrick_actuators_12_central.mat')
input_central = reshape(cell2mat(inputs),20,50);
input_cost_ref = 0;
input_cost_central= 0;
for i = 1:50
    input_cost_ref = input_cost_ref + 1000*input_ref(:,i)'*input_ref(:,i);
    input_cost_central = input_cost_central + 1000*input_central(:,i)'*input_central(:,i);
end
input_cost_ref
input_cost_central
figure
hold on
plot(tvec(1:end-1),input_ref(9,:),'--','color',cm(5,:))
plot(tvec(1:end-1),input_central(9,:),'color',cm(4,:))


figure 
hold on
files = [1, 4];
legend_str = {};
for k=1:2
    j = files(k);
    load(strcat('patrick_contact_forces_12_',filenames{j},'.mat'))
    force_cell = horzcat(contacts{:});
    Z = cellfun(@(x)reshape(x,1,1,[]),force_cell([9,13,15],:),'un',0);
    arm_force = cell2mat(Z)./0.05;
    summed_arm_forces = squeeze(sum(arm_force,1));
    arm_tip_force = cell2mat(force_cell(15,:))./0.05;
    plot(tvec(2:end),summed_arm_forces(2:end,1),'Linewidth',1.3, 'color', cm(j,:))
    %legend_str{end+1} = strcat(weight_pct{j});
    plot(tvec(2:end),vecnorm(summed_arm_forces(2:end,2:4)'),'--','Linewidth',1.3, 'color', cm(j,:),'HandleVisibility','off')
    %legend_str{end+1} = strcat(weight_pct{j},' frictional force');
    
    total_arm_force(k) = trapz(tvec(2:end),vecnorm(summed_arm_forces(2:end,2:4)'));
end
leg = legend(weight_pct{1},weight_pct{4});
total_arm_force
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',14)
leg.FontSize = 10;


figure
hold on
for k=1:2
    j = files(k);
    load(strcat('patrick_contact_forces_12_',filenames{j},'.mat'))
    force_cell = horzcat(contacts{:});
    Z = cellfun(@(x)reshape(x,1,1,[]),force_cell([1:8, 10:12, 14, 16:31],:),'un',0);
    body_force = cell2mat(Z)./0.05;
    summed_body_forces = squeeze(sum(body_force,1));
    plot(tvec(2:end),summed_body_forces(2:end,1),'Linewidth',1.3, 'color', cm(j,:))
    %legend_str{end+1} = strcat(weight_pct{j});
    plot(tvec(2:end),vecnorm(summed_body_forces(2:end,2:4)'),'--','Linewidth',1.3, 'color', cm(j,:),'HandleVisibility','off')
    %legend_str{end+1} = strcat(weight_pct{j},' frictional force');
    total_body_force(k) = trapz(tvec(2:end),vecnorm(summed_body_forces(2:end,2:4)'));
end
leg = legend(weight_pct{1},weight_pct{4});
total_body_force
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',14)
leg.FontSize = 10;

%%
clear num_zero distance
figure
hold on
tvec = 0:0.05:10;
files = 1:4;
cm = lines(7);
c_i = 9;
for k=1:4
    j = files(k);
    load(strcat('patrick_contacts_nominal_',filenames{j},'.mat'))
    load(strcat('patrick_maximal_state_nominal_',filenames{j},'.mat'))
    for i=1:length(contact_inds)
        contact = contacts{contact_inds(i)};
        num_zero{j,i} = sum(contact<0.0001);
%         stairs(contact(2:end))
%         legend_str{i} = num2str(bodies(i));
    end
    num_zero{j,i+1} = sum(sum([contacts{16:31}]<0.0001));
    contact = contacts{c_i};
    legend_str{k} = weight_pct{j};
    plot(tvec,contact,'Linewidth',1.3,'color',cm(j,:))
    body_1_mat = [state{1}{:}];
    distance(j) = -body_1_mat(1,end)+body_1_mat(1,1);
end
num_zero
distance
leg = legend(legend_str);

title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Leg tip vertical distance (cm)')
set(gca,'FontSize',14)

figure 
hold on
for j=1:4
    load(strcat('patrick_maximal_state_nominal_',filenames{j},'.mat'))
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'Linewidth',1.3)
    legend_str{j} = weight_pct{j};
end
leg = legend(legend_str);

title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Distance (cm)')
set(gca,'FontSize',14)

figure 
hold on
for j=1:4
    load(strcat('patrick_maximal_state_nominal_',filenames{j},'.mat'))
    body_1_mat = [state{5}{:}];
    plot(tvec,(-body_1_mat(1,:))*100,'Linewidth',1.3)
    legend_str{j} = weight_pct{j};
end
leg = legend(legend_str);

title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Distance (cm)')
set(gca,'FontSize',14)


figure 
hold on
files = [1, 4];
legend_str = {};
for k=1:2
    j = files(k);
    load(strcat('patrick_contact_forces_nominal_',filenames{j},'.mat'))
    force_cell = horzcat(contacts{:});
    Z = cellfun(@(x)reshape(x,1,1,[]),force_cell([9,15],:),'un',0);
    arm_force = cell2mat(Z)./0.05;
    summed_arm_forces = squeeze(sum(arm_force,1));
    arm_tip_force = cell2mat(force_cell(15,:))./0.05;
    plot(tvec(2:end),summed_arm_forces(2:end,1),'Linewidth',1.3, 'color', cm(j,:))
    %legend_str{end+1} = strcat(weight_pct{j});
    plot(tvec(2:end),vecnorm(summed_arm_forces(2:end,3:4)'),'--','Linewidth',1.3, 'color', cm(j,:),'HandleVisibility','off')
    %legend_str{end+1} = strcat(weight_pct{j},' frictional force');
    
    total_arm_force(k) = trapz(tvec(2:end),vecnorm(summed_arm_forces(2:end,2:4)'));
end
leg = legend(weight_pct{1},weight_pct{4});
total_arm_force
ylim([0 1.0])
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',14)
leg.FontSize = 10;


figure
hold on
for k=1:2
    j = files(k);
    load(strcat('patrick_contact_forces_nominal_',filenames{j},'.mat'))
    force_cell = horzcat(contacts{:});
    Z = cellfun(@(x)reshape(x,1,1,[]),force_cell([1:8, 10:14, 16:31],:),'un',0);
    body_force = cell2mat(Z)./0.05;
    summed_body_forces = squeeze(sum(body_force,1));
    plot(tvec(2:end),summed_body_forces(2:end,1),'Linewidth',1.3, 'color', cm(j,:))
    %legend_str{end+1} = strcat(weight_pct{j});
    plot(tvec(2:end),vecnorm(summed_body_forces(2:end,3:4)'),'--','Linewidth',1.3, 'color', cm(j,:),'HandleVisibility','off')
    %legend_str{end+1} = strcat(weight_pct{j},' frictional force');
    total_body_force(k) = trapz(tvec(2:end),vecnorm(summed_body_forces(2:end,2:4)'));
end
leg = legend(weight_pct{1},weight_pct{4});
total_body_force
ylim([0 1.5])
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',14)
leg.FontSize = 10;



%% support polygon
close all
hat = @(w) [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

% figure 
% hold on
files = [1, 4];
legend_str = {};
SP = {};
body = {};
contact_inds = 6:15;
for k=1:2
    j = files(k);
    load(strcat('patrick_contacts_nominal_',filenames{j},'.mat'))
    load(strcat('patrick_maximal_state_nominal_all_',filenames{j},'.mat'))
    load(strcat('patrick_contact_location_nominal_',filenames{j},'.mat'))
    for i=1:length(contact_inds)
        contact = contacts{contact_inds(i)};
        contact_binary(i,:) = contact<0.0001;
        body_state = horzcat(state{:});
        index = (contact_inds(i)-5)*13-1;
        x = body_state(index+1:index+3, :);
        q = body_state(index+4:index+7, :);
        v = body_state(index+8:index+10, :);
        w = body_state(index+11:index+12, :);
        Q = [];
        for t = 1:length(q)
            qt = q(:,t);
            L = [qt(1) -qt(2:4).'; qt(2:4) qt(1)*eye(3,3) + hat(qt(2:4))];
            R = [qt(1) -qt(2:4).'; qt(2:4) qt(1)*eye(3,3) - hat(qt(2:4))];
            H = [zeros(1,3); eye(3,3)];
            Q = [Q; H.'*L*R*H];
        end
%         contact_location{i} = x + reshape(Q*[0;0;-0.13/2],3,[]) - [0;0;1]*0.032/2;
    end
    fails = [];
    for t = 1:length(q)
        contacts = [];
        if contact_binary(1,t) || contact_binary(2,t)
            fails = [fails t];
        end
        for i = 1:length(contact_inds) 
            if contact_binary(i,t)
                
                contacts = [contacts, [contact_location{t}{contact_inds(i)}]];
            end
        end
        support{t} = contacts;
        % support_alt(t)
    end
    t_fail{k} = fails;
%     num_zero{j,i+1} = sum(sum([contacts{16:31}]<0.0001));
    figure
    hold on
    h = plot(0,0,'o', 'MarkerSize' ,5, 'MarkerFaceColor', 'b');
    h2 = plot(state{1}(1),state{1}(2),'o', 'MarkerSize' ,5, 'MarkerFaceColor', 'r');
    win_size = 1.5;
    xlim([-win_size, win_size]);
    ylim([-win_size, win_size]);
    SP_k = [];
    for i=2:length(support)
        if ~isempty(support{i})
            support{i}(:,abs(support{i}(2,:)) < 1e-5) = [];
            support{i}(:,- state{i}(1) + support{i}(1,:) > 0.05) = [];
            if ~isempty(support{i})
                if all(state{i}(1) > support{i}(1,:))
                    SP_k = [SP_k, support{i}(1,1)];
                else
                    SP_k = [SP_k, support{i}(1,1)];
                end
            else
                SP_k = [SP_k, SP_k(end)];
            end
            % set(h, 'XData', support{i}(1,:));
            % set(h, 'YData', support{i}(2,:));
            % set(h2, 'XData', state{i}(1));
            % set(h2, 'YData', state{i}(2));
            % drawnow;
            % pause(0.05);
        else
            SP_k = [SP_k, SP_k(end)];
        end
    end
    SP{k} = SP_k;
    b = horzcat(state{:});
    b = b(1,:);
    body{k} = b;
%     contact = contacts{c_i};
%     legend_str{k} = weight_pct{j};
%     plot(tvec,contact,'Linewidth',1.3,'color',cm(j,:))
%     body_1_mat = [state{1}{:}];
%     distance(j) = -body_1_mat(1,end)+body_1_mat(1,1);
    t_fail{k}(diff(t_fail{k}) > 10)

end
tvec = 0:0.05:10;
close all
figure
cm = lines(7);
plot(tvec(2:end),- SP{1},'--','Linewidth',1.3,'color',cm(1,:))
hold on
plot(tvec(2:end),- body{1}(2:end),'-','Linewidth',1.3,'color',cm(1,:))
legend({"Support Polygon", "Body Position"})
xlabel('Time (s)')
ylabel('X Position (cm)')
set(gca,'FontSize',14)

figure
plot(tvec(2:end),- SP{2},'--','Linewidth',1.3,'color',cm(4,:))
hold on
plot(tvec(2:end),- body{2}(2:end),'-','Linewidth',1.3,'color',cm(4,:))
legend({"Support Polygon", "Body Position"})
xlabel('Time (s)')
ylabel('X Position (cm)')
set(gca,'FontSize',14)

t_fail{1}(diff(t_fail{1}) > 10,'-','Linewidth',1.3,'color',cm(j,:))
% leg = legend(weight_pct{1},weight_pct{4});
% total_arm_force
% ylim([0 1.0])
% title(leg,'% mass in limbs')
% xlabel('Time (s)')
% ylabel('Force (N)')
% set(gca,'FontSize',14)
% leg.FontSize = 10;

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                LONG LEGS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
filenames_long = {'dist','bstar', 'rob', 'central'};
weight_pct = {'98%','75%','55%','14%'}; 
contact_inds = [8,14];
clear num_zero distance
figure
hold on
tvec = 0:0.05:10;
files = 1:4;
cm = lines(7);
c_i = 9;
for k=1:4
    j = files(k);
    load(strcat('patrick_contacts_nominal_long_',filenames_long{j},'.mat'))
    load(strcat('patrick_maximal_state_nominal_all_long_',filenames_long{j},'.mat'))
    for i=1:length(contact_inds)
        contact = contacts{contact_inds(i)};
        num_zero{j,i} = sum(contact<0.0001);
%         stairs(contact(2:end))
%         legend_str{i} = num2str(bodies(i));
    end
    num_zero{j,i+1} = sum(sum([contacts{16:31}]<0.0001));
    contact = contacts{c_i};
    legend_str{k} = weight_pct{j};
    plot(tvec,contact,'Linewidth',1.3,'color',cm(j,:))
    body_1_mat = [state{1}{:}];
    distance(j) = -body_1_mat(1,end)+body_1_mat(1,1);
end
num_zero
distance
leg = legend(legend_str);

title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Leg tip vertical distance (cm)')
set(gca,'FontSize',14)

figure 
hold on
for j=1:4
    load(strcat('patrick_maximal_state_nominal_all_long_',filenames_long{j},'.mat'))
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'Linewidth',1.3)
    legend_str{j} = strcat(weight_pct{j}," long");
end
cm = lines(7);
for j=1:4
    load(strcat('patrick_maximal_state_nominal_',filenames{j},'.mat'))
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'--','Linewidth',1.3,'color',cm(j,:))
    legend_str{j+4} = strcat(weight_pct{j}," short");
end
leg = legend(legend_str);

title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Distance (cm)')
set(gca,'FontSize',14)


figure 
hold on
files = [1, 2];
legend_str = {};
for k=1:2
    j = files(k);
    load(strcat('patrick_contact_forces_nominal_long_',filenames_long{j},'.mat'))
    force_cell = horzcat(contacts{:});
    Z = cellfun(@(x)reshape(x,1,1,[]),force_cell([9,15],:),'un',0);
    arm_force = cell2mat(Z)./0.05;
    summed_arm_forces = squeeze(sum(arm_force,1));
    arm_tip_force = cell2mat(force_cell(15,:))./0.05;
    plot(tvec(2:end),summed_arm_forces(2:end,1),'Linewidth',1.3, 'color', cm(j,:))
    %legend_str{end+1} = strcat(weight_pct{j});
    plot(tvec(2:end),vecnorm(summed_arm_forces(2:end,3:4)'),'--','Linewidth',1.3, 'color', cm(j,:),'HandleVisibility','off')
    %legend_str{end+1} = strcat(weight_pct{j},' frictional force');
    
    total_arm_force(k) = trapz(tvec(2:end),vecnorm(summed_arm_forces(2:end,2:4)'));
end
leg = legend(weight_pct{1},weight_pct{4});
total_arm_force
ylim([0 1.0])
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',14)
leg.FontSize = 10;


figure
hold on
for k=1:2
    j = files(k);
    load(strcat('patrick_contact_forces_nominal_long_',filenames_long{j},'.mat'))
    force_cell = horzcat(contacts{:});
    Z = cellfun(@(x)reshape(x,1,1,[]),force_cell([1:8, 10:14, 16:31],:),'un',0);
    body_force = cell2mat(Z)./0.05;
    summed_body_forces = squeeze(sum(body_force,1));
    plot(tvec(2:end),summed_body_forces(2:end,1),'Linewidth',1.3, 'color', cm(j,:))
    %legend_str{end+1} = strcat(weight_pct{j});
    plot(tvec(2:end),vecnorm(summed_body_forces(2:end,3:4)'),'--','Linewidth',1.3, 'color', cm(j,:),'HandleVisibility','off')
    %legend_str{end+1} = strcat(weight_pct{j},' frictional force');
    total_body_force(k) = trapz(tvec(2:end),vecnorm(summed_body_forces(2:end,2:4)'));
end
leg = legend(weight_pct{1},weight_pct{4});
total_body_force
ylim([0 1.5])
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',14)
leg.FontSize = 10;

figure 
hold on
for j=1:length(filenames_long)
    load(strcat('patrick_contacts_nominal_long_',filenames_long{j},'.mat'))
    load(strcat('patrick_maximal_state_nominal_all_long_',filenames_long{j},'.mat'))
    for i=1:length(contact_inds)
        contact = contacts{contact_inds(i)};
        num_zero{j,i} = sum(contact<0.0001);
%         stairs(contact(2:end))
%         legend_str{i} = num2str(bodies(i));
    end
    num_zero{j,i+1} = sum(sum([contacts{16:31}]<0.0001));
    body_5_mat = [state{5}{:}];
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_5_mat(1,:)+body_1_mat(1,:))*100,'Linewidth',1.3)
    legend_str{j} = filenames_long{j};
    distance(j) = -body_1_mat(1,end)+body_1_mat(1,1);
end
num_zero
distance
legend(legend_str)
xlim([0 10])
xlabel('Time (s)')
ylabel('Front arm x location relative to body (cm)')


%%
close all
tvec = 0:0.05:2.5;
files = [1, 2,3,4];
figure 
hold on
%%%%%%%%%%%%%%%%%%% LONG %%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:4
    j = files(k);
    load(strcat('patrick_maximal_state_distr_long_',filenames_long{j},'.mat'))
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'Linewidth',1.3)
    legend_str{j} = strcat(weight_pct{j}," long");
    load(strcat('patrick_actuators_distr_long_',filenames_long{j},'.mat'))
    costs{j,1} = sum([inputs{:}].^2,2)*100000;
    arm_movement{j} = sum(abs(diff([state{9}{:}]-[state{1}{:}],1,2)),2);

end
%%%%%%%%%%%%%%%%%%%%% SHORT %%%%%%%%%%%%%%%%%%%%%%%%%%
cm = lines(7);
for j=1:length(filenames)
    load(strcat('patrick_maximal_state_h_',filenames{j},'.mat'))
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_1_mat(1,1:51)+body_1_mat(1,1))*100,'--','Linewidth',1.3,'color',cm(j,:))
    legend_str{j+4} = strcat(weight_pct{j}," short");
end

% for k=1:3
%     j = files(k);
%     load(strcat('patrick_maximal_state_distr_long_',filenames_long{j},'.mat'))
%     body_1_mat = [state{1}{:}];
%     plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'--','Linewidth',1.3)
%     legend_str{j+3} = weight_pct{j};
%     load(strcat('patrick_actuators_distr_long_',filenames_long{j},'.mat'))
%     costs{j,2} = sum(vecnorm([inputs{:}]).^2);
% end
% load(strcat('patrick_maximal_state_ref.mat'))
% body_1_mat = [state{1}{:}];
% plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'--','Linewidth',1.3)
% legend_str{j+1} = 'ref';
leg = legend(legend_str);

title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Distance (cm)')
set(gca,'FontSize',14)

figure
hold on
files = [1, 2];
contact_inds = [9,15];
cm = lines(7);
for k=1:2
    j = files(k);
    load(strcat('patrick_contacts_distr_long_',filenames_long{j},'.mat'))
    load(strcat('patrick_maximal_state_distr_long_',filenames_long{j},'.mat'))
    for i=1:length(contact_inds)
        contact = contacts{contact_inds(i)};
        num_zero{j,i} = sum(contact<0.0001);
%         stairs(contact(2:end))
%         legend_str{i} = num2str(bodies(i));
    end
    num_zero{j,i+1} = sum(sum([contacts{16:31}]<0.0001));
    body_5_mat = [state{5}{:}];
    body_1_mat = [state{1}{:}];
    plot(tvec,(-body_5_mat(1,:)+body_1_mat(1,:))*100,'Linewidth',1.3)
    legend_str{j} = weight_pct{j};
    distance(j) = -body_1_mat(1,end)+body_1_mat(1,1);
end
num_zero
distance
legend(legend_str)
xlabel('Time (s)')
ylabel('Front arm x location relative to body (cm)')

figure
hold on
files = [1, 2];
cm = lines(7);
c_i = 15;
for k=1:2
    j = files(k);
    load(strcat('patrick_contacts_distr_long_',filenames_long{j},'.mat'))
    load(strcat('patrick_maximal_state_distr_long_',filenames_long{j},'.mat'))
    contact = contacts{c_i};
    legend_str{k} = weight_pct{j};
    plot(tvec,contact,'Linewidth',1.3,'color',cm(j,:))
end
load(strcat('patrick_contacts_ref.mat'))
contact = contacts{c_i};
plot(tvec,contact,'--','Linewidth',1.3,'color',cm(5,:))
legend_str{k+1} = 'ref';
leg = legend(legend_str);
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Leg tip vertical distance (cm)')
set(gca,'FontSize',14)


load(strcat('patrick_actuators_ref.mat'))
input_ref = reshape(cell2mat(inputs),20,50);
load('patrick_actuators_12_central.mat')
input_central = reshape(cell2mat(inputs),20,50);
input_cost_ref = 0;
input_cost_central= 0;
for i = 1:50
    input_cost_ref = input_cost_ref + 1000*input_ref(:,i)'*input_ref(:,i);
    input_cost_central = input_cost_central + 1000*input_central(:,i)'*input_central(:,i);
end
input_cost_ref
input_cost_central
figure
hold on
plot(tvec(1:end-1),input_ref(9,:),'--','color',cm(5,:))
plot(tvec(1:end-1),input_central(9,:),'color',cm(4,:))


figure 
hold on
files = [1, 2];
legend_str = {};
for k=1:2
    j = files(k);
    load(strcat('patrick_contact_forces_12_long_',filenames_long{j},'.mat'))
    force_cell = horzcat(contacts{:});
    Z = cellfun(@(x)reshape(x,1,1,[]),force_cell([5,11],:),'un',0);
    arm_force = cell2mat(Z)./0.05;
    summed_arm_forces = squeeze(sum(arm_force,1));
    arm_tip_force = cell2mat(force_cell(15,:))./0.05;
    plot(tvec(2:end),summed_arm_forces(2:51,1),'Linewidth',1.3, 'color', cm(j,:))
    %legend_str{end+1} = strcat(weight_pct{j});
    plot(tvec(2:end),vecnorm(summed_arm_forces(2:51,2:4)'),'--','Linewidth',1.3, 'color', cm(j,:),'HandleVisibility','off')
    %legend_str{end+1} = strcat(weight_pct{j},' frictional force');
    
    total_arm_force(k) = trapz(tvec(2:end),vecnorm(summed_arm_forces(2:51,2:4)'));
end
leg = legend(weight_pct{1},weight_pct{4});
total_arm_force
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',14)
leg.FontSize = 10;


figure
hold on
for k=1:2
    j = files(k);
    load(strcat('patrick_contact_forces_12_long_',filenames_long{j},'.mat'))
    force_cell = horzcat(contacts{:});
    Z = cellfun(@(x)reshape(x,1,1,[]),force_cell([1:8, 10:12, 13:14, 16:31],:),'un',0);
    body_force = cell2mat(Z)./0.05;
    summed_body_forces = squeeze(sum(body_force,1));
    plot(tvec(2:end),summed_body_forces(2:51,1),'Linewidth',1.3, 'color', cm(j,:))
    %legend_str{end+1} = strcat(weight_pct{j});
    plot(tvec(2:end),vecnorm(summed_body_forces(2:51,2:4)'),'--','Linewidth',1.3, 'color', cm(j,:),'HandleVisibility','off')
    %legend_str{end+1} = strcat(weight_pct{j},' frictional force');
    total_body_force(k) = trapz(tvec(2:end),vecnorm(summed_body_forces(2:51,2:4)'));
end
leg = legend(weight_pct{1},weight_pct{2});
total_body_force
title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Force (N)')
set(gca,'FontSize',14)
leg.FontSize = 10;

%% Velocity
close all
tvec = 0:0.05:2.5;
files = [1, 4];
figure 
hold on
%%%%%%%%%%%%%%%%%%% LONG %%%%%%%%%%%%%%%%%%%%%%%%%
% for k=1:4
%     j = files(k);
%     load(strcat('patrick_maximal_state_distr_long_',filenames_long{j},'.mat'))
%     body_1_mat = [state{1}{:}];
%     plot(tvec(2:end),diff((-body_1_mat(1,:)+body_1_mat(1,1)))*100,'Linewidth',1.3)
%     legend_str{j} = strcat(weight_pct{j}," long");
%     load(strcat('patrick_actuators_distr_long_',filenames_long{j},'.mat'))
%     costs{j,1} = sum([inputs{:}].^2,2)*100000;
%     arm_movement{j} = sum(abs(diff([state{9}{:}]-[state{1}{:}],1,2)),2);
% 
% end
%%%%%%%%%%%%%%%%%%%%% SHORT %%%%%%%%%%%%%%%%%%%%%%%%%%
cm = lines(7);
for i=1:length(files)
    j = files(i);
    load(strcat('patrick_maximal_state_h_',filenames{j},'.mat'))
    body_1_mat = [state{1}{:}];
    plot(tvec(2:end),diff((-body_1_mat(1,1:51)+body_1_mat(1,1)))*100,'-','Linewidth',1.3,'color',cm(j,:))
    legend_str{j+4} = strcat(weight_pct{j}," short");
end

% for k=1:3
%     j = files(k);
%     load(strcat('patrick_maximal_state_distr_long_',filenames_long{j},'.mat'))
%     body_1_mat = [state{1}{:}];
%     plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'--','Linewidth',1.3)
%     legend_str{j+3} = weight_pct{j};
%     load(strcat('patrick_actuators_distr_long_',filenames_long{j},'.mat'))
%     costs{j,2} = sum(vecnorm([inputs{:}]).^2);
% end
% load(strcat('patrick_maximal_state_ref.mat'))
% body_1_mat = [state{1}{:}];
% plot(tvec,(-body_1_mat(1,:)+body_1_mat(1,1))*100,'--','Linewidth',1.3)
% legend_str{j+1} = 'ref';
leg = legend(weight_pct{1},weight_pct{4});

title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Velocity (cm/s)')
set(gca,'FontSize',14)

figure
hold on
cm = lines(7);
for i=1:length(files)
    j = files(i);
    load(strcat('patrick_maximal_state_nominal_',filenames{j},'.mat'))
    body_1_mat = [state{1}{:}];
    plot(tvec(2:end),diff((-body_1_mat(1,52:102)+body_1_mat(1,1)))*100,'-','Linewidth',1.3,'color',cm(j,:))
    legend_str{j+4} = strcat(weight_pct{j}," short");
end

leg = legend(weight_pct{1},weight_pct{4});

title(leg,'% mass in limbs')
xlabel('Time (s)')
ylabel('Velocity (cm/s)')
set(gca,'FontSize',14)
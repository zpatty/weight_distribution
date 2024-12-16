clear
close all

names = {'C-star'; 'B-star'; 'KK-star'; 'A-star'; 'H-star'};
whole_body_dry = [2.66;8.61;7.374; 8.75; 2.135];
whole_body_wet = [2.04; 6.31; 5.02; nan; 1.424];

arms = {};
discs = {};
arms_density = zeros(5,5);
discs_density = zeros(5,1);
% [dry suspended proximal-dry distal-dry]...
arms{1} = [0.355  0.235 0.266 0.083;
    0.389 0.275 0.283 0.077;
    0.341 0.251 0.260 0.081;
    0.314 0.213 0.244 0.064;
    0.336 0.224 0.258 0.064];

% [dry suspended]
discs{1} = [0.689; 0.563; nan; nan];
whole{1} = [2.66; 2.04; nan; nan];

arms{2} = [1.324 0.857 1.128 0.177;
    1.373 0.983 0.836 0.212;
    1.324 0.86 1.074 0.239;
    1.282 0.83 1.068 0.176;
    1.072 0.692 0.898 0.118];
discs{2} = [2.063; 1.695; nan; nan];
whole{2} = [8.61; 6.31; nan; nan];

arms{3} = [1.021 0.663 0.91 0.1;
    0.976 0.64 0.864 0.087;
    1.04 0.669 0.889 0.13;
    1.467 0.921 1.31 0.166;
    1.279 0.837 1.111 0.148];
discs{3} = [1.516; 1.215; nan; nan];
whole{3} = [7.374; 5.02; nan; nan];


arms{4} = [0.867 0.544 0.759 0.106;
    0.934 0.577 0.792 0.123;
    1.481 0.893 1.289 0.149;
    1.797 1.085 1.089 0.659;
    1.572 0.952 1.234 0.325];
discs{4} = [2.003; 1.498; nan; nan];
whole_body_wet(4) = sum(arms{4}(:,2)) + 1.498;
whole{4} = [8.75; whole_body_wet(4); nan; nan];


arms{5} = [0.268 0.181 0.242 0.024;
    0.337 0.224 0.293 0.036;
    0.452 0.29 0.375 0.079;
    0.157 0.104 0.128 0.032;
    0.36 0.242 0.314 0.041];
discs{5} = [0.426; 0.326; nan; nan];
whole{5} = [2.135; 1.424; nan; nan];



densities = whole_body_dry./whole_body_wet

raw = [whole{1}'; arms{1}; discs{1}'; whole{2}'; arms{2}; discs{2}'; whole{3}'; arms{3}; discs{3}'; whole{4}'; arms{4}; discs{4}'; whole{5}'; arms{5}; discs{5}'];
raw_table.data = raw;
raw_table.dataFormat = {'%.3f'};
raw_latex = latexTable(raw_table)
fid=fopen('raw_table.tex','w');
[nrows,ncols] = size(raw_latex);
for row = 1:nrows
    fprintf(fid,'%s\n',raw_latex{row,:});
end
fclose(fid);
fprintf('\n... your LaTex code has been saved as ''MyLatex.tex'' in your working directory\n');


for i = 1:5
    arms_density(:,i) = arms{i}(:,1)./arms{i}(:,2);
    discs_density(i) = discs{i}(1)/discs{i}(2);
    percentage_mass_arms(i) = sum(arms{i}(:,1))/(sum(arms{i}(:,1))+discs{i}(1));
    percentage_weight_arms(i) = sum(arms{i}(:,2))/(sum(arms{i}(:,2))+discs{i}(2));
    percentage_weight_arms(i) = 1 - sum(discs{i}(2))/whole_body_wet(i);
    percentage_proximal(i) = mean(arms{i}(:,3)./arms{i}(:,1));
    percentage_distal(i) = mean(arms{i}(:,4)./arms{i}(:,1));
end

mean_arms_density = mean(arms_density,1)
mean_discs_density = mean(discs_density)

percentage_mass_arms
percentage_weight_arms
percentage_proximal
percentage_distal

whole_body_avg_density = mean(densities)
whole_body_std_density = std(densities)
avg_arm_density = mean(arms_density,'all')
std_arm_density = std(arms_density,0,'all')
avg_discs_density = mean(discs_density)
std_discs_density = std(discs_density)
avg_percentage_mass_arms = mean(percentage_mass_arms)
std_percentage_mass_arms = std(percentage_mass_arms)
avg_percentage_weight_arms = mean(percentage_weight_arms)
std_percentage_weight_arms = std(percentage_weight_arms)
avg_percentage_mass_proximal = mean(percentage_proximal)
std_percentage_mass_proximal = std(percentage_proximal)
avg_percentage_mass_distal = mean(percentage_distal)
std_percentage_mass_distal = std(percentage_distal)

data_table=table([1:5]',densities, mean_arms_density', discs_density, percentage_mass_arms', percentage_weight_arms')
X = round([[1:5]',densities, mean_arms_density', discs_density, percentage_mass_arms', percentage_weight_arms'],2);
input.data = X;
latexTable(input)
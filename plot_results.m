function plot_results()
a=jsondecode(fileread('results/analysis.json'));conditions={'blur','dark','occlusion'};modes={'fixed_low','calibrated_constant','calibrated_quality'};rows=a.rows;if ~iscell(rows),rows=num2cell(rows);end;errors=zeros(3,3);coverage=errors;colors=[.35 .35 .35;0 .35 .7;.1 .55 .25];
for i=1:3,for j=1:3,k=find(cellfun(@(q)strcmp(q.condition,conditions{i})&&strcmp(q.name,modes{j}),rows));errors(i,j)=100*mean(cellfun(@(q)q.xy_final_m,rows(k)));coverage(i,j)=100*mean(cellfun(@(q)q.observation_coverage_95,rows(k)));end,end
f=figure('Visible','off','Color','w','Position',[100 100 1200 520]);cl=onCleanup(@() close(f));
subplot(1,2,1);b=bar(errors);for j=1:3,b(j).FaceColor=colors(j,:);end;set(gca,'XTickLabel',conditions);grid on;ylabel('mean final horizontal distance (cm)');title('Untouched seeds 444 / 555');legend(modes,'Interpreter','none','Location','northwest');
subplot(1,2,2);b=bar(coverage);for j=1:3,b(j).FaceColor=colors(j,:);end;hold on;yline(95,'k--','95% reference');ylim([0 105]);set(gca,'XTickLabel',conditions);grid on;ylabel('observations inside predicted 95% radius (%)');title('Covariance calibration on new closed-loop data');
st=sgtitle('SYNTHETIC: fit on 111/222/333, freeze, test on 444/555','Color','k');erc_plot_style(f);set(st,'Color','k');exportgraphics(f,'results/calibrated_comparison.png','Resolution',145,'BackgroundColor','white');
end

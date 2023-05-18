fig = figure;

numIterations = max(find(packing.historyBest > 0));

plot(1:numIterations, packing.historyBest(1:numIterations), 1:numIterations, packing.historyAvg(1:numIterations), 'linewidth', 1)

legend({'种群中最佳','种群的平均'},'Location','northeast')

xlabel('迭代次数')
ylabel('目标函数')

set(gca,'fontsize',12,'fontname','等线')

sum(packing.historyTime)
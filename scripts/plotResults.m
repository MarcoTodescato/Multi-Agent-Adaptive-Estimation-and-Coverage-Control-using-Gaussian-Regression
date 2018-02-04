
% Multi-agents adaptive estimation and coverage control using Gaussian regression 
%
% Copyright (C) 2017, University of Padova
% Andrea Carron , carrona@ethz.ch
% Marco Todescato, mrc.todescato@gmail.com
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

%% Coverage Plot
PositionRecursiveEstimation = node2pos(RecursiveEstimation.Node);
NodeTrue = computeMomenta(RecursiveEstimation.Node,Parameter,Parameter.TrueFunction);
Centroid = zeros(Parameter.NumberOfAgents,2);
NodeNoEstimate = zeros(Parameter.NumberOfAgents,2);

for i=1:Parameter.NumberOfAgents
    Centroid(i,:) = NodeTrue(i).C;
    NodeNoEstimate(i,:) = Parameter.NoEstimation.Node(i).pos;
end

figure
hold on
axis([Parameter.GridStart Parameter.GridEnd Parameter.GridStart Parameter.GridEnd]);
VarianceHandler = contour(Parameter.Xgrid,Parameter.Ygrid,RecursiveEstimation.DensityFunction,20,'LineWidth',1);
[vx,vy] = voronoi(PositionRecursiveEstimation(:,1),PositionRecursiveEstimation(:,2));
VoroniLineHandler = plot(vx,vy,'k-');
set(VoroniLineHandler,'LineWidth',1.5);
NodePositionHandler = plot(PositionRecursiveEstimation(:,1),PositionRecursiveEstimation(:,2),'.r','MarkerSize',25);
CentroidHandler = plot(Centroid(:,1),Centroid(:,2),'.b','MarkerSize',20);
%NodeNoEstimateHandler = plot(NodeNoEstimate(:,1),NodeNoEstimate(:,2),'.g','MarkerSize',20);
box on
set(gca,'XTick',[])
set(gca,'YTick',[])
hold off

%% Posterior Variance
figure
hold on
Hmax = plot(RecursiveEstimation.maxAPosterioriVariance);
Have = plot(RecursiveEstimation.averageAPosterioriVariance,'--');
Hmin = plot(RecursiveEstimation.minAPosterioriVariance,'.-');
hold off
grid on
axis([0 Parameter.Niter 0 1])
set(Hmax,'LineWidth',2,'color',rgb('Indigo'));
set(Have,'LineWidth',2,'color',rgb('DarkGreen'));
set(Hmin,'LineWidth',2,'color',rgb('Crimson'));
legendHandler = legend('$V_{\max}$','$V_{\rm ave}$','$V_{\min}$');
set(legendHandler,'FontSize',12,'Interpreter','Latex');
xAxisHandler = xlabel('Iteration');
yAxisHandler = ylabel('Posterior Variance');
set(xAxisHandler,'FontSize',12);
set(yAxisHandler,'FontSize',12);
pbaspect([1 0.5 1])
set(gcf, 'PaperPositionMode', 'auto');

%% Cost function Behaviour 
figure
hold on
HNO = plot(Parameter.NoEstimation.CostFunction,'--');
HRE = plot(RecursiveEstimation.CostFunction);
grid on
set(HNO,'LineWidth',2,'color',rgb('Indigo'));
set(HRE,'LineWidth',2,'color',rgb('DarkGreen'));
legendHandler = legend('Coverage','Estimation + Coverage');
set(legendHandler,'FontSize',13);
xAxisHandler = xlabel('Iteration');
yAxisHandler = ylabel('Cost Function');
set(xAxisHandler,'FontSize',12);
set(yAxisHandler,'FontSize',12);
hold off
pbaspect([1 0.5 1])

%% Number of Exploring agents
figure
NEA = stem(RecursiveEstimation.NumberOfAgentsExploring);
grid on
set(NEA,'LineWidth',2,'color',rgb('OrangeRed'));
set(legendHandler,'FontSize',13);
xAxisHandler = xlabel('Iteration');
yAxisHandler = ylabel('Number of Exploring Agents');
set(xAxisHandler,'FontSize',12);
set(yAxisHandler,'FontSize',12);
hold off
pbaspect([1 0.5 1])


%% Average Energy Used
figure
hold on
AERE = plot(RecursiveEstimation.averageEnergy);
AENE = plot(Parameter.NoEstimation.averageEnergy,'--');
grid on
set(AERE,'LineWidth',2,'color',rgb('Indigo'));
set(AENE,'LineWidth',2,'color',rgb('DarkGreen'));
legendHandler = legend('Coverage','Estimation + Coverage');
set(legendHandler,'FontSize',13);
xAxisHandler = xlabel('Iteration');
yAxisHandler = ylabel('Average Energy');
set(xAxisHandler,'FontSize',12);
set(yAxisHandler,'FontSize',12);
hold off
pbaspect([1 0.5 1])

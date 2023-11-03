function multiple_xy_graph(block)
setup(block);

function setup(block)
block.NumInputPorts=1;
block.NumDialogPrms=2;
block.RegBlockMethod('Start',@Start);
block.RegBlockMethod('Outputs',@Outputs);
block.RegBlockMethod('Terminate',@Terminate);

function Start(block)
colors=lines;
cla; box on
for i=1:block.DialogPrm(1).Data
    eval(['global h' num2str(i)])
    eval(['h' num2str(i) '=animatedline(''color'',[' num2str(colors(i,:)) ']);'])
end
axis(block.DialogPrm(2).Data)

function Outputs(block)
for i=1:block.DialogPrm(1).Data
    eval(['global h' num2str(i)])
    x=['block.InputPort(1).Data(' num2str(2*i-1) ')'];
    y=['block.InputPort(1).Data(' num2str(2*i) ')'];
    eval(['addpoints(h' num2str(i) ',' x ',' y ');' ])
end
drawnow

function Terminate(block)
for i=1:block.DialogPrm(1).Data
    eval(['clear global h' num2str(i)])
end
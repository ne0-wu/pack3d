figure;

for i = 1:(size(offsetPoints,2) - 1)
    x = posPoints((offsetPoints(i) + 1):offsetPoints(i + 1),1);
    y = posPoints((offsetPoints(i) + 1):offsetPoints(i + 1),2);
    z = posPoints((offsetPoints(i) + 1):offsetPoints(i + 1),3);
    T = vidTriangles((offsetTriangles(i) + 1):offsetTriangles(i + 1),:);

    randColor = rand(1,3);
    trimesh(T,x,y,z,'EdgeColor',[0 0 0],'FaceColor',randColor)
    hold on
    drawnow;
    
%     pause(1);
end

axis equal
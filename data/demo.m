% This is a script to generate illustrations for SI

vertices= readmatrix('stampfli24_vertices.txt');

%%
figure(1)
hold on
axis off 
axis equal

for i = 1:length(vertices)
    face = reshape(vertices(i,:),2,[])';
    fill(face(:,1), face(:,2), [255 229 204]/255, 'LineWidth',1.5);
end
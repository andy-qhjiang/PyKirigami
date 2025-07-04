% generate the vertices file and constraints file for Tangram

% contracted pattern
face_sets = {};
face_sets{1} = [0 0 0 2 0 0 1 1 0];
face_sets{2} = [2 0 0 4 0 0 4 2 0];
face_sets{3} = [2 0 0 3 1 0 2 2 0 1 1 0];
face_sets{4} = [3 1 0 3 3 0 2 2 0];
face_sets{5} = [3 1 0 4 2 0 4 4 0 3 3 0];
face_sets{6} = [0 0 0 2 2 0 0 4 0];
face_sets{7} = [2 2 0 4 4 0 0 4 0];

% write the vertices info into a text file
writecell(face_sets', 'tangram_vertices.txt', 'Delimiter', ' ');

%% plot seven faces
figure(1)
clf
axis equal
hold on
for i=1:7
    points = reshape(face_sets{i}, 3, numel(face_sets{i})/3)';
    fill3(points(:, 1), points(:, 2), points(:, 3), [255 229 204]/255, 'LineWidth', 1.5);
end





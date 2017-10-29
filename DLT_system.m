function A = DLT_system(u, x)
    A = [];
    for i=1:length(u)
        A = [A ; DLT_point2vec(u(i,:), x(i,:))];
    end
end

function A = DLT_point2vec(u, x)
    A = kron(eye(2), [x,1]);
    A = [A , -u' * [x,1]];
end
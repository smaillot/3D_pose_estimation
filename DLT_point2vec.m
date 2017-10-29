function A = DLT_point2vec(u, x)
    A = kron(eye(2), [x,1]);
    A = [A , -u' * [x,1]];
end
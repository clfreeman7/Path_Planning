n_experiments = 5;
n_states = 16;
euler_tours = zeros(n_experiments, n_states*(n_states-1)+1);

for i = 1:n_experiments
    euler_tours(i,:) = randtour(n_states);
end

writematrix(euler_tours, 'euler_tours.txt')

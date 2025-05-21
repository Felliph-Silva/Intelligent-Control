% Limpar variáveis e figuras
clear;
close all;

% Carregar os dados do arquivo
dados = readmatrix('controle_velocidade_motor_ex.txt'); % Lê o arquivo de dados

% Extrair colunas (assumindo a estrutura: [tempos, referencias, velocidades, erros, pwms])
tempos = dados(:, 1);      % Tempo (s)
referencias = dados(:, 2); % Velocidade de referência (RPM)
velocidades = dados(:, 3); % Velocidade atual (RPM)
erros = referencias - velocidades;       % Erro (RPM)

% Calcular os critérios de desempenho
erro_absoluto = abs(erros);
erro_quadratico = erros.^2;

% ISE - Integral do Erro Quadrático
ISE = trapz(tempos, erro_quadratico);
ISE_acumulado = cumtrapz(tempos, erro_quadratico);

% IAE - Integral do Erro Absoluto
IAE = trapz(tempos, erro_absoluto);
IAE_acumulado = cumtrapz(tempos, erro_absoluto);

% ITAE - Integral do Tempo*Erro Absoluto
ITAE = trapz(tempos, tempos.*erro_absoluto);
ITAE_acumulado = cumtrapz(tempos, tempos.*erro_absoluto);

% ITSE - Integral do Tempo*Erro Quadrático
ITSE = trapz(tempos, tempos.*erro_quadratico);
ITSE_acumulado = cumtrapz(tempos, tempos.*erro_quadratico);

% Plotar todos os critérios
figure;

% ISE
subplot(4, 2, 1);
plot(tempos, erro_quadratico, 'r', 'LineWidth', 1.5);
title('Erro Quadrático (ISE)');
ylabel('Erro² (RPM²)');
grid on;

subplot(4, 2, 2);
plot(tempos, ISE_acumulado, 'b', 'LineWidth', 1.5);
title('ISE Acumulado');
ylabel('ISE (RPM²·s)');
grid on;

% IAE
subplot(4, 2, 3);
plot(tempos, erro_absoluto, 'm', 'LineWidth', 1.5);
title('Erro Absoluto (IAE)');
ylabel('|Erro| (RPM)');
grid on;

subplot(4, 2, 4);
plot(tempos, IAE_acumulado, 'c', 'LineWidth', 1.5);
title('IAE Acumulado');
ylabel('IAE (RPM·s)');
grid on;

% ITAE
subplot(4, 2, 5);
plot(tempos, tempos.*erro_absoluto, 'g', 'LineWidth', 1.5);
title('Tempo*Erro Absoluto (ITAE)');
ylabel('t*|Erro| (RPM·s)');
grid on;

subplot(4, 2, 6);
plot(tempos, ITAE_acumulado, 'k', 'LineWidth', 1.5);
title('ITAE Acumulado');
ylabel('ITAE (RPM·s²)');
grid on;

% ITSE
subplot(4, 2, 7);
plot(tempos, tempos.*erro_quadratico, 'y', 'LineWidth', 1.5);
title('Tempo*Erro Quadrático (ITSE)');
xlabel('Tempo (s)');
ylabel('t*Erro² (RPM²·s)');
grid on;

subplot(4, 2, 8);
plot(tempos, ITSE_acumulado, 'Color', [0.5 0 0.5], 'LineWidth', 1.5);
title('ITSE Acumulado');
xlabel('Tempo (s)');
ylabel('ITSE (RPM²·s²)');
grid on;

% Exibir valores no console
fprintf('Critérios de Desempenho:\n');
fprintf('ISE: %.2f RPM²·s\n', ISE);
fprintf('IAE: %.2f RPM·s\n', IAE);
fprintf('ITAE: %.2f RPM·s²\n', ITAE);
fprintf('ITSE: %.2f RPM²·s²\n\n', ITSE);
% Limpar workspace e fechar figuras
clear;
close all;

% Nome do arquivo de dados
nomeArquivo = 'controle_velocidade_motor.txt';

% Verifica se o arquivo existe
if exist(nomeArquivo, 'file') == 2
    % Lê os dados do arquivo
    dados = readmatrix(nomeArquivo);

    % Extrai as colunas dos dados
    tempos = dados(:, 1);          % Tempo (s)
    referencias = dados(:, 2);     % Referência de velocidade (RPM)
    velocidades = dados(:, 3);     % Velocidade atual do motor (RPM)
    erros = referencias - velocidades;          % Sinal de erro (RPM)
    pwms = dados(:, 4);            % Sinal de PWM

    % Plota os gráficos
    figure(1);
    plot(tempos, referencias, 'b', 'DisplayName', 'Referência de Velocidade');
    hold on;
    plot(tempos, velocidades, 'r', 'DisplayName', 'Velocidade Atual');
    title('Controle de Velocidade do Motor');
    xlabel('Tempo (s)');
    ylabel('Velocidade (RPM)');
    legend;
    grid on;

    figure(2)
    subplot(2, 1, 1);
    plot(tempos, erros, 'g', 'DisplayName', 'Sinal de Erro');
    title('Sinal de Erro');
    xlabel('Tempo (s)');
    ylabel('Erro (RPM)');
    legend;
    grid on;

    subplot(2, 1, 2);
    plot(tempos, pwms, 'm', 'DisplayName', 'Sinal de PWM');
    title('Sinal de PWM');
    xlabel('Tempo (s)');
    ylabel('Valor de PWM');
    ylim(0, 300);
    legend;
    grid on;

else
    % Mensagem de erro se o arquivo não existir
    error('Arquivo não encontrado: %s', nomeArquivo);
end
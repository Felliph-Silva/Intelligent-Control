% Limpar leituras e gráficos anteriores
clear;
close all;

% Configuração da porta serial
portaSerial = '/dev/ttyUSB0'; % Substitua pela sua porta serial (exemplo: COM3 no Windows)
taxaBits = 115200;            % Taxa de bits definida no código Arduino

% Abre a conexão serial
s = serialport(portaSerial, taxaBits);

% Inicializa a figura com subplots
figure(1);
subplot(2, 1, 1); % Subplot para o erro
h1 = animatedline('Color', 'b', 'DisplayName', 'Erro');
title('Erro de Velocidade');
xlabel('Tempo (s)');
ylabel('Erro (RPM)');
grid on;
legend;

subplot(2, 1, 2); % Subplot para o PWM
h2 = animatedline('Color', 'r', 'DisplayName', 'PWM');
title('Sinal PWM');
xlabel('Tempo (s)');
ylabel('PWM');
ylim([0, 260]); % Faixa fixa para o eixo Y
grid on;
legend;

% Tempo de execução
tempoInicial = datetime('now');
duration = seconds(inf); % Executa indefinidamente

% Inicializa arrays para armazenar dados de tempo, erro e PWM
tempos = [];
referencias = [];
velocidades = [];
erros = [];
pwms = [];

% Usa try-catch para garantir que os dados sejam salvos mesmo em caso de interrupção
try
    % Leitura e plotagem em tempo real
    while datetime('now') - tempoInicial < duration
    if s.NumBytesAvailable > 0
        % Lê os valores do Arduino
        data = readline(s); % Lê uma linha da porta serial
        valores = str2double(split(data, ' ')); % Divide pelos separadores de espaço

        % Verifica se os dados são válidos
        if numel(valores) == 3
            referencia = valores(1);      % Referência de velocidade (RPM)
            velocidade = valores(2);      % Velocidade atual do motor (RPM)
            pwm = valores(3);             % Sinal PWM

            % Calcula o erro
            erro = referencia - velocidade;

            % Obtém o tempo atual
            tempoAtual = datetime('now') - tempoInicial;
            tempoSegundos = seconds(tempoAtual);

            % Adiciona dados aos arrays
            tempos = [tempos; tempoSegundos];
            referencias = [referencias; referencia];
            velocidades = [velocidades;referencias];
            erros = [erros; erro];
            pwms = [pwms; pwm];

            % Adiciona pontos aos gráficos
            addpoints(h1, tempoSegundos, erro); % Subplot do erro
            addpoints(h2, tempoSegundos, pwm);  % Subplot do PWM
            drawnow;
            end
        end
    end
catch ME
    % Captura a interrupção (por exemplo, fechar a janela do gráfico)
    disp('Execução interrompida pelo usuário.');
end

% Fecha a conexão serial
clear s;

% Salva os dados em um arquivo
dadosSalvos = [tempos, referencias, velocidades, pwms];
writematrix(dadosSalvos, 'controle_velocidade_motor1.txt', 'Delimiter', '\t');

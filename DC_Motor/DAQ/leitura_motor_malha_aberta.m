% Limpar variáveis e figuras
clear;
close all;

% Configuração da porta serial
portaSerial = '/dev/ttyACM0'; % Substitua pela sua porta serial
taxaBits = 115200;            % Taxa de bits definida no código Arduino

% Abre a conexão serial
s = serialport(portaSerial, taxaBits);

% Criar uma interface gráfica
fig = figure('Name', 'Controle de Motor em Malha Aberta', 'NumberTitle', 'off', ...
             'Position', [100, 100, 800, 600]);

% Inicializa UserData com os campos necessários
fig.UserData.s = s;               % Objeto serial
fig.UserData.motorLigado = false; % Estado do motor
fig.UserData.tempos = [];         % Array de tempos
fig.UserData.referencias = [];    % Array de referências
fig.UserData.velocidades = [];    % Array de velocidades
fig.UserData.pwms = [];           % Array de PWMs
fig.UserData.tempoInicial = datetime('now'); % Tempo inicial

% Botão virtual para ligar/desligar o motor
btnStart = uicontrol('Style', 'pushbutton', 'String', 'Ligar Motor', ...
                     'Position', [400, 30, 100, 50], 'Callback', @toggleMotor);

% Texto para exibir a velocidade do motor
txtVelocidade = uicontrol('Style', 'text', 'String', 'Velocidade: 0 RPM', ...
                          'Position', [30, 30, 140, 50], 'FontSize', 12);

% Inicializa os gráficos
h1 = animatedline('Color', 'r', 'DisplayName', 'Velocidade Atual');
legend;
title('Controle de Velocidade do Motor');
xlabel('Tempo (s)');
ylabel('Velocidade (RPM)');
grid on;

% Função de callback para o botão
function toggleMotor(src, ~)
    % Acessa os dados da figura
    fig = ancestor(src, 'figure');
    s = fig.UserData.s;
    
    % Alterna o estado do motor
    fig.UserData.motorLigado = ~fig.UserData.motorLigado;
    
    if fig.UserData.motorLigado
        set(src, 'String', 'Desligar Motor'); % Muda o texto do botão
        write(s, '1', 'char'); % Envia comando para ligar o motor
    else
        set(src, 'String', 'Ligar Motor'); % Muda o texto do botão
        write(s, '0', 'char'); % Envia comando para desligar o motor
    end
end

% Loop principal para leitura e plotagem
try
    while ishandle(fig)
        if fig.UserData.motorLigado
            if fig.UserData.s.NumBytesAvailable > 0
                % Lê os dados do Arduino
                data = readline(fig.UserData.s);
                valores = str2double(split(data, ' '));
                
                if numel(valores) == 3
                    % Extrai os valores
                    referencia = valores(1);
                    velocidade = valores(2);
                    pwm = valores(3);
                    
                    % Calcula o tempo decorrido
                    tempoAtual = seconds(datetime('now') - fig.UserData.tempoInicial);
                    
                    % Atualiza os arrays
                    fig.UserData.tempos = [fig.UserData.tempos; tempoAtual];
                    fig.UserData.referencias = [fig.UserData.referencias; referencia];
                    fig.UserData.velocidades = [fig.UserData.velocidades; velocidade];
                    fig.UserData.pwms = [fig.UserData.pwms; pwm];
                    
                    % Atualiza o gráfico
                    addpoints(h1, tempoAtual, velocidade);
                    
                    % Atualiza o texto da velocidade
                    set(txtVelocidade, 'String', ['Velocidade: ', num2str(velocidade), ' RPM']);
                    
                    drawnow;
                end
            end
        end
        pause(0.01); % Pausa mínima para atualização
    end
catch ME
    % Captura interrupções e salva os dados
    disp('Execução interrompida pelo usuário.');
end

% Fecha a conexão serial e salva os dados
clear s;
dadosSalvos = [fig.UserData.tempos, fig.UserData.referencias, fig.UserData.velocidades, fig.UserData.pwms];
writematrix(dadosSalvos, 'motor_malha_fechada.txt', 'Delimiter', '\t');
disp('Dados salvos com sucesso.');
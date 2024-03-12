%%　アニメーションの特徴を考慮したゲイン設計
clear all               %変数の初期化

%% GIFから画像を抽出&画像データの読み込み (5.2.1節)
gifFilename = 'test_4.gif';
info = imfinfo(gifFilename, 'gif');                                         % GIF画像の情報を取得
imagedata = cell(1, numel(info));                                           % 画像データを格納するセル配列を初期化 

% 1つずつフレームを読み込んで保存しないで画像データを取得
for k = 1:numel(info)
    frame = imread(gifFilename, 'Frames', k);                               % 各フレームの画像を読み取る
    imagedata{k} = frame;                                                   % 画像データをセル配列に格納

    % フレーム間の画素値の差の平均を計算
    if k > 1
        diff = abs(double(imagedata{k}) - double(imagedata{k-1}));
        MADs(k-1) = mean(diff, 'all');
    end
end

%　配列の平均値を最初に追加する
MAD_mean = mean(MADs);
MADs = [MAD_mean, MADs];
disp('画像データが読み込まれました。');  % 保存は行われず、画像データの読み込みが完了したことを通知
disp('MAD:');
disp(MADs);

%% GIFから画像を抽出&画像データの読み込み (5.2.2節)
% gifFilename = 'man.gif';
% info = imfinfo(gifFilename, 'gif');                                         % GIF画像の情報を取得
% imagedata = cell(1, numel(info));                                           % 画像データを格納するセル配列を初期化 
% all_grid_MADs = zeros(3, 3, numel(info));                                 % 各領域の平均値を格納する配列を初期化
% 
% % 1つずつフレームを読み込んで保存しないで画像データを取得
% for k = 1:numel(info)
%     frame = imread(gifFilename, 'Frames', k);                               % 各フレームの画像を読み取る
%     imagedata{k} = frame;                                                   % 画像データをセル配列に格納
% 
%     % フレーム間の画素値の差の平均を計算（空間的な手法）
%     if k > 1
%         diff = abs(double(imagedata{k}) - double(imagedata{k-1}));          % フレーム間の画像値の差
%         [height, width] = size(frame);                                      %　画像のサイズ
% 
%        %フレーム画像を3x3の領域に分割
%         h_step = floor(height / 3);
%         w_step = floor(width / 3);
%         grid_MADs = zeros(3, 3);
% 
%         for i = 1:3
%             for j = 1:3
%                 %各エリアの境界の決定
%                 row_start = (i-1)*h_step + 1;
%                 row_end = i*h_step;
%                 col_start = (j-1)*w_step + 1;
%                 col_end = j*w_step;
% 
%                 % 境界にある場合
%                 if i == 3
%                     row_end = height;
%                 end
%                 if j == 3
%                     col_end = width;
%                 end
% 
%                 % 各エリアの平均を計算する
%                 region_diff = diff(row_start:row_end, col_start:col_end);
%                 grid_MADs(i, j) = mean(region_diff(:), 'all');
%             end
%         end
%         all_grid_MADs(:,:,k-1) = grid_MADs;
%     end
% end
% 
% %平均値を最初に追加
% mean_grid_MADs = mean(all_grid_MADs, 3);
% all_grid_MADs = cat(3, mean_grid_MADs, all_grid_MADs);
% 
% disp(all_grid_MADs);
% disp('画像データが読み込まれました。');  % 保存は行われず、画像データの読み込みが完了したことを通知



%% システムパラメータの設定
n=3000;                 %ロボットの数
KP=6;                   %フィードバックゲイン
KD=0.5;
KI=0.25;
e_pre=zeros(n,2);
ie=zeros(n,2);
Q=[100, 100];            %フィールドの大きさ[x,y]

% 数値計算のため，フィールドを微小要素に分割
dx=Q(1)/250;            %微小要素の大きさ（x方向，小さくするほど正確に計算できるが，計算時間がかかる）
dy=Q(2)/250;            %微小要素の大きさ（y方向，小さくするほど正確に計算できるが，計算時間がかかる）
dq=dx*dy;               %微小要素の面積

% ボロノイ領域の計算に必要なパラメータ
% 半径R_iからR_mまで徐々に広げながら計算を行う
% 詳細なアルゴリズムについては修論の参考文献 9)Coverage control for mobile sensing networks を参照のこと
R_i=2;                  %ボロノイ領域計算時の初期半径
R_m=200;                %ボロノイ領域計算時の最大半径（フィールドよりも大きくしておけばOK）

%% 重み関数の生成
% 格納行列の作成
fig_data = cell(1, numel(info));
varphi = cell(1, numel(info));
phi = cell(1, numel(info));
imgsize = cell(1, numel(info), 2);

% 画像データの行列を作成
for k = 1:numel(info)
    %imagedata{k} = uint8(0.2989*imagedata{k}(:,:,1)+0.5870*imagedata{k}(:,:,2)+0.1140*imagedata{k}(:,:,3));     %カラーから白黒へ（白黒画像の場合は必要なし）   
    fig_data{k} = double(imagedata{k});                                                                          %データ型の変換
    varphi{k} = flipud(fig_data{k})/255;                                                                         %正規化（画素値を0から1に）＋白黒反転（黒が1で白が0）＋行列の上下反転
    phi{k} = exp(10*(varphi{k}-1));                         %exp(10*(varphi{k}-1))                                                     %重み関数の生成、黒い場所ほど相対的に大きな値をとるようにする
    [imgsize{1,k,1}, imgsize{1,k,2}]  = size(imagedata{k});                                                      %画像のサイズを獲得 [height, width, channels]
end

clear imagedata                                                                                                  %メモリ節約のため，不要なデータをクリア

%% 初期配置の設定
% 初期位置による差異を避けるために同じ初期位置を使用
load init_pos.mat initial_pos
x0 = initial_pos(1:n)';
y0 = initial_pos(n+1:2*n)';

%% シミュレーション（初期位置を設定するため）
% 準備
fr=numel(info);                             %フレーム数
st=10;                                      %ステップ数
dt=0.1;                                     %ステップ幅(サンプル時間[sec])
Data=zeros(fr,st,2*n);                      %シミュレーション結果（座標の時系列データ）を保存する配列
Distance=zeros(st*fr,n,2);                  %2つのstep間、ロボットの移動距離の合計

tic;                                        %シミュレーションに要する時間の計測開始
for frame = 1:fr
    if frame ==1
        X=x0;                                                              %ロボット群のx座標の初期化                                     
        Y=y0;                                                              %ロボット群のy座標の初期化 
    else
        X=squeeze(Data((frame-1),st,1:n));                                 %ロボット群のx座標; size(X)=(3000,1)
        Y=squeeze(Data((frame-1),st,(n+1):2*n));                           %ロボット群のy座標; size(Y)=(3000,1)
    end
    
    for ite=1:st

        % データの保存（各フレーム100ステップ）
        Data(frame,ite, 1:n)=X';                                        %ロボット群のx座標
        Data(frame,ite, (n+1):2*n)=Y';                                  %ロボット群のy座標 

        % 制御入力の初期化
        u=zeros(n,2);                                                   %制御入力（x,y方向）
         
        % 制御則（ロボットごとに制御入力を計算する）
        for i=1:n
            % ロボットiのボロノイ領域を求める
            R=R_i;                                                      %ボロノイ領域を計算する半径の初期化
            [v,x_l,y_u,q_max]=voro([X,Y],R,i,Q,dx,dy);                  %関数ファイルvoro.mによるボロノイ領域の計算
    
                % ボロノイ領域が確定するまで逐次計算
                while ((R<2*q_max)||(q_max==0))&&(R<R_m)
                    R=2*R;                                              %計算する領域の半径を広げる
                    [v,x_l,y_u,q_max]=voro([X,Y],R,i,Q,dx,dy);          %関数ファイルvoro.mによるボロノイ領域の計算
                end
           
           % 制御入力の計算
           [v_y,v_x]=find(v==1);                                                    %ボロノイ領域（=配列vの要素が1）のインデックスを取り出す
           if isempty(v_y)==0                                                       %ボロノイ領域が存在する場合のみ制御入力を計算する
                x=(x_l+v_x-2)*dx+dx/2;                                              %インデックスを座標に変換（x座標）
                y=(y_u+v_y-2)*dy+dy/2;                                              %インデックスを座標に変換（y座標）
                Phi=diag(phi{1,frame}(ceil(y/(Q(2)/imgsize{1,frame,1})),ceil(x/(Q(1)/imgsize{1,frame,2}))));        %ボロノイ領域における重み関数の値を取り出す
                cent=dq*[sum(x.*Phi),sum(y.*Phi)]/(dq*sum(Phi));                    %ボロノイ領域の重み付き重心
                e(i,:) = cent-[X(i),Y(i)];                                          % 誤差ベクトル

                % ロボットのいるエリアを求める
                region_x = min(floor(X(i)/Q(1)*3)+1, 3);
                region_y = min(floor(Y(i)/Q(2)*3)+1, 3);
                Diff = MADs(frame);                                                 % このフレームに対応する画素値の差の平均値（卒論の5.2.1節）
                % Diff = all_grid_MADs(region_x, region_y, frame);                    % このエリアに対応する画素値の差の平均値（空間的な手法　卒論の5.2.2節）

           %　空間的な手法を用いた制御則
                Kp = 0.3*(Diff)-4.5+6;                                              % 6にマイナスプラス1.5
                u(i,:) = Kp*e(i,:);    
           end
        end
        
        % ロボット群の移動
        X=X+u(:,1)*dt;              %x方向
        Y=Y+u(:,2)*dt;              %y方向

        if mod(ite,10)==0
            fprintf('ite=%d, frame=%d\n', ite, frame);   %現在のステップ数，フレーム数を出力
        end
        epre=e;
    end


end

% save 'loop_4' Data;			%変数-Data-をmatファイルで保存
toc;                                %シミュレーションに要する時間の計測終了

%% 結果の保存
% ロボット（■で表示する）の大きさの設定
a=1/(1*10^4);                   %フィールドの大きさとロボット1個の大きさの比
A=a*Q(1)*Q(2);                   %ロボットの具体的な大きさ

outputFolder = 'gain_design\man\';
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder); 
end

for frame = 1:fr
    for step = 1:st
        h = figure('visible','off');
        set(h, 'Position', [100, 100, 400, 400]);
        hold on                                                                        %重ね書きをする
        axis([0 Q(1) 0 Q(2)]);                                                         %座標軸の範囲（フィールドの大きさに対応）
        axis off                                                                       %目盛の非表示
        X0=squeeze(Data(frame,step,1:n));                                              %ロボット群のx座標
        Y0=squeeze(Data(frame,step,(n+1):end));                                        %ロボット群のy座標
        X1=X0';
        Y1=Y0';
        X2=[X1-sqrt(A)/2;X1-sqrt(A)/2;X1+sqrt(A)/2;X1+sqrt(A)/2];       %ロボットを四角形で表したときの四隅のx座標（y座標と対応）
        Y2=[Y1+sqrt(A)/2;Y1-sqrt(A)/2;Y1-sqrt(A)/2;Y1+sqrt(A)/2];       %ロボットを四角形で表したときの四隅のy座標（x座標と対応）
        patch(X2,Y2,'k','Edgecolor','none');                                           %ロボット群の描画（黒の四角形）
        set(gca,'Position',get(gca,'OuterPosition'));                                  %余白を切り取る
        fileName = fullfile(outputFolder, ['frame_' num2str(frame) '_step_' num2str(step) '.png']);
        print(h, fileName, '-dpng', '-r96');
        hold off
        close(h);
    end
    frame
end
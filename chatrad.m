clear all               %変数の初期化

%% GIFから画像を抽出&画像データの読み込み
gifFilename = 'test_4.gif';
info = imfinfo(gifFilename, 'gif');  % GIF画像の情報を取得

imagedata = cell(1, numel(info));    %  画像データを格納するセル配列を初期化 

% 1つずつフレームを読み込んで保存しないで画像データを取得
for k = 1:numel(info)
    frame = imread(gifFilename, 'Frames', k);
    imagedata{k} = frame;  % 画像データをセル配列に格納
end

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

%% 初期位置の作成と保存
% % ロボット群の初期配置の保存(フィールド上にロボット群を一様に分布させる)
% x0=Q(1)*rand(n,1);                          %初期配置のx座標
% y0=Q(2)*rand(n,1);                          %初期配置のy座標
%
% initial_pos=zeros(n*2);
% initial_pos(1:n) = x0';
% initial_pos(n+1:2*n) = y0';
% 
% save 'init_pos.mat' initial_pos

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
                d = sqrt((cent(1)-X(i))^2+(cent(2)-Y(i))^2);                        %　誤差ベクトルのノルム
                de = (e(i,:)-e_pre(i,:))/dt;
                ie(i,:) = ie(i,:) + (e(i,:)+e_pre(i,:))*dt/2;

           % PID制御則
                Sig = (-(1+exp(-10*d+3)).^(-1)+1.8);
                u(i,:) = Sig*KP*e(i,:)+ KI*ie(i,:)+ KD*de;                  %制御入力の計算   
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

outputFolder = 'chat_result\test4\sig';
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

disp('すべての画像がフォルダに保存された');


%% アニメーションの作成と保存(.mp4)
% % ロボット（■で表示する）の大きさの設定
% a = 1/(1*10^4);               % フィールドの大きさとロボット1個の大きさの比
% A = a * Q(1) * Q(2);          % ロボットの具体的な大きさ
% 
% % MP4 ファイルを作成
% mov = VideoWriter('test4_sig_80fps.mp4', 'MPEG-4');
% mov.FrameRate = 80;
% open(mov);
% 
% % 描画面の設定
% figure                      % 描画面の生成
% hold on                     % 重ね書きをする
% axis equal                  % 座標軸の縦横の長さを揃える
% axis([0 Q(1) 0 Q(2)]);      % 座標軸の範囲（フィールドの大きさに対応）
% axis off                    % 目盛の非表示
% 
% % フィールドの描画
% patch([0 0 Q(1) Q(1)], [Q(2) 0 0 Q(2)], 'w', 'EdgeColor', 'k', 'Linewidth', 1)
% 
% fprintf('start\n');
% % アニメーション
% for frame = 1:fr  
%     for ite = 1:st       % 1:size(Data,2)
%         % 現在の状態を取得
%         X0 = squeeze(Data(frame, ite, 1:n));            % ロボット群のx座標
%         Y0 = squeeze(Data(frame, ite, (n+1):end));      % ロボット群のy座標
%         X1 = X0';
%         Y1 = Y0';
%         X2 = [X1-sqrt(A)/2; X1-sqrt(A)/2; X1+sqrt(A)/2; X1+sqrt(A)/2];   % ロボットを四角形で表したときの四隅のx座標（y座標と対応）
%         Y2 = [Y1+sqrt(A)/2; Y1-sqrt(A)/2; Y1-sqrt(A)/2; Y1+sqrt(A)/2];   % ロボットを四角形で表したときの四隅のy座標（x座標と対応）
% 
%         % ロボット群を点と四角形で描画（plotを入れないとaviファイルがうまくできない）
%         p1 = plot(X1, Y1, 'k.', 'MarkerSize', 0.5, 'MarkerFaceColor', 'none');  % ロボット群の描画（黒の点）
%         p2 = patch(X2, Y2, 'k', 'Edgecolor', 'none');                         % ロボット群の描画（黒の四角形）
%         drawnow                                                             % おまじない的なもの
%         pause(0.001)                                                        % 少しストップ
% 
%         % フレームの取得とファイルへの追加
%         M = getframe;                                                     % フレームの取得
%         writeVideo(mov, M);
% 
%         % 新しいものを描くために，いま描いたものを消す
%         if ite ~= size(Data,2)
%             delete(p1)			                                        % 描画したものを消す
% 		    delete(p2)			                                        % 描画したものを消す
%         end
%     end
%     if frame ~= size(Data,1)
%         delete(p1)
%         delete(p2)
%     end
% end
% 
% % AVIファイルへの書き込みを終了して閉じる
% close(mov);


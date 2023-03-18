% TMP_N = N;
TMP_N = 11;
SIM_TIME = CAVs{TMP_N}.tf+CAVs{TMP_N}.t0;
N_T = 4*SIM_TIME;
t = linspace(0,SIM_TIME,N_T);
colors = parula(TMP_N);
scale = 1;
axis square
figure(1);

fig_len = 2*L*scale+2*rw;
center = fig_len/2;

path2_len = 2*(L+rw);
path2 = @(s) [fig_len-s*fig_len/(path2_len), center+rw/2];
path5_len = 2*(L+rw);
path5 = @(s) [center-rw/2,fig_len-s*fig_len/(path2_len)];
path8_len = 2*(L+rw);
path8 = @(s) [s*fig_len/(path2_len), center-rw/2];
path11_len = 2*(L+rw);
path11 = @(s) [center+rw/2,s*fig_len/(path2_len)];

% paths = [@path1,path2,path5,path8,path11];
marker=8;
for i = 300:N_T
    figure(1)
    drawFig(L*scale,rw);
    
    
    axis square
    view_size = 60;
%     top = center+view_size;
%     xlim([center-view_size,center+view_size]);
%     ylim([center-view_size,center+view_size]);
    top = fig_len;
    xlim([0*fig_len,1*fig_len]);
    ylim([0*fig_len,1*fig_len]);
    text(top,top, [num2str(t(i), '%02.2f'), ' seconds'], ...
                'HorizontalAlignment', 'right');
            
    for n=74:75%TMP_N
        if t(i) >= CAVs{n}.t0 && t(i) <= CAVs{n}.t0 + CAVs{n}.tf
            time = t(i)-CAVs{n}.t0;
            if n == 28
                continue;
            end
            phi = CAVs{n}.phis;
            s = phi(1)*time^3+phi(2)*time^2+phi(3)*time;
            if CAVs{n}.path == 2
                P = path2(s);
            elseif CAVs{n}.path == 5
                P = path5(s);
            elseif CAVs{n}.path == 8
                P = path8(s);
            elseif CAVs{n}.path == 11
                P = path11(s);
            else
                continue;
            end
%             disp(phi);
            plot(P(1),P(2), 'o', ...
                'MarkerFaceColor', colors(n,:), 'Color', 'k', ...
                'MarkerSize', marker);
%             fprintf('CAV %d: (x,y) = (%.2f, %.2f)\n',n,P(1),P(2));
            
        end
    end
end

% function P = path1(s)
%     L = 200;    rw = 3.5;
%     TOT = 2*(L+rw); center = L+rw;
%     if s <= 200
%         x = TOT-s;  y = center +0.5*rw;
%     elseif s <= L+0.5*pi*0.5*rw
%         x = 
%     end
%     P = [x,y];
% end



function drawFig(L,rw)
    clf;
    hold on;
    
    SIZE = 2*rw+2*L;
    center = SIZE/2;

    %draw a green background
    bg = [0.7, 0.9, 0.7];
    rectangle('Position', [0,0,SIZE,SIZE], 'FaceColor', bg, 'EdgeColor', bg);
    
    
    fd = 5;%(p1(1,2)+p3(1,2))/2 - p1(1,2); %fog delta (distance from centerline)   
    
    %grey road background
    grey = [0.3, 0.3, 0.3];
    grey = [40, 43, 42]/255 + 0.2;
    %horizontal road
    rectangle('Position', [0,center-rw,SIZE,2*rw], 'FaceColor', grey, 'EdgeColor', grey);
    %vertical road
    rectangle('Position', [center-rw,0,2*rw,SIZE], 'FaceColor', grey, 'EdgeColor', grey);
    
    plot([center,center],[0,L],'-y');
    plot([center,center],[L+2*rw,SIZE],'-y');
    plot([0,L],[center,center],'-y');
    plot([L+2*rw,SIZE],[center,center],'-y');
    
end
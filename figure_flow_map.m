% load sample_m30_v9.mat     % Load results
load myColorMap.mat

%%

D = 20;     % Distance between center of intersections
nR = V;
Rx = zeros([nR,1]);
Ry = zeros([nR,1]);

for i = 1:nRow
    for j = 1:nCol
        r = (i-1)*nCol+j;
        Rx(r) = (j-1)*D;
        Ry(r) = D*(nRow-i);
    end
end

%% Plot

%%%%%%%%%%%%%% Figure parameters %%%%%%%%%%%%%%
eps_x = 1.5;
eps_y = 2;
e = 1;
lw = 5;
dev = [0 D/2 e e; -e -e 0 D/2; 0 -D/2 -e -e; e e 0 -D/2;
       0 D/2 -e -e; e e 0 D/2; 0 -D/2 e e; -e -e 0 -D/2];

figure(100);
cm = colormap(myColorMap);
hold on;

ax = gca;
ax.Box = 'on';
ax.XTick =[];
ax.YTick =[];

colLen = D*nCol;
rowLen = D*nRow;
xlim([-20,colLen]);
ylim([-20,rowLen]);

MAX_VAL = max(sum(X,1));
caxis([0 MAX_VAL]);
colorbar;

for r = 1:nR
    for dir = 1:8
        val = fix(256*sum(X(:,8*(r-1)+dir))/MAX_VAL);
        if val == 0
            val = 1;
        end
        plot([Rx(r)+dev(dir,1) Rx(r)+dev(dir,2)],[Ry(r)+dev(dir,3) Ry(r)+dev(dir,4)],'LineWidth',lw,'Color',cm(val,:));
        
    end
end

for i=1:nR
        patch([Rx(i)-eps_x,Rx(i)+eps_x,Rx(i)+eps_x,Rx(i)-eps_x],[Ry(i)-eps_y,Ry(i)-eps_y,Ry(i)+eps_y,Ry(i)+eps_y],'w');
end

for r = 1:nR
    for dir = 1:8
        p=nsidedpoly(1000,'Center',[Rx(r)+dev(dir,2),Ry(r)+dev(dir,4)],'Radius',1);
        plot(p,'FaceColor','w','FaceAlpha',1,'LineWidth',1.5);
    end
end
for r = 1:nR
    for dir = 1:8
        p=nsidedpoly(1000,'Center',[Rx(r)+dev(dir,2),Ry(r)+dev(dir,4)],'Radius',1);
        origin = false;
        destination = false;
        for m=1:M
            if o(m,1)==r && o(m,2)==dir
                origin = true;
            end
            if d(m,1)==r && d(m,2)==dir
                destination = true;
            end
        end
        if origin || destination
            plot(p,'FaceColor','k','FaceAlpha',1,'LineWidth',1.5);
        end
    end
end

%%
for m = 5:25
    figure(100+m);
    cm = colormap(myColorMap);
    hold on;
    ax = gca;
    ax.Box = 'on';
    ax.XTick =[];
    ax.YTick =[];

    colLen = D*nCol;
    rowLen = D*nRow;
    xlim([-20,colLen]);
    ylim([-20,rowLen]);

    MAX_VAL = max(X(m,:));
    caxis([0 MAX_VAL]);
    colorbar;

    for r = 1:nR
        for dir = 1:8
            val = fix(256*(X(m,8*(r-1)+dir))/MAX_VAL);
            if val == 0
                val = 1;
            end
            plot([Rx(r)+dev(dir,1) Rx(r)+dev(dir,2)],[Ry(r)+dev(dir,3) Ry(r)+dev(dir,4)],'LineWidth',lw,'Color',cm(val,:));

        end
    end

    for i=1:nR
            patch([Rx(i)-eps_x,Rx(i)+eps_x,Rx(i)+eps_x,Rx(i)-eps_x],[Ry(i)-eps_y,Ry(i)-eps_y,Ry(i)+eps_y,Ry(i)+eps_y],'w');
    end
    for r = 1:nR
        for dir = 1:8
            p=nsidedpoly(1000,'Center',[Rx(r)+dev(dir,2),Ry(r)+dev(dir,4)],'Radius',1);
            plot(p,'FaceColor','w','FaceAlpha',1);
        end
    end
    for r = 1:nR
        for dir = 1:8
            p=nsidedpoly(1000,'Center',[Rx(r)+dev(dir,2),Ry(r)+dev(dir,4)],'Radius',1);
            origin = false;
            destination = false;
            if o(m,1)==r && o(m,2)==dir
                origin = true;
            end
            if d(m,1)==r && d(m,2)==dir
                destination = true;
            end

            if origin || destination
                plot(p,'FaceColor','k','FaceAlpha',1);
            end
        end
    end

end

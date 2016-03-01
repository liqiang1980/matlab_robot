tablerobot = mdl_tablerobot();
Q = [0,0,0.2];
tablerobot.plot(Q,'workspace', [-1 1 -1 1 -1 1]);
tablerobot.teach();
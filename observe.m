%% �ж�ϵͳ�Ƿ��

function observe(A, B, C,D)
    % ����ϵͳ״̬�ռ�ģ��
    sys = ss(A,B,C,D);

    % ����ɹ۲��Ծ���
    O = obsv(sys);

    % �ж�ϵͳ�Ƿ�ɹ۲�
    if rank(O) == size(A,1)
        disp('The system is observable.');
    else
        disp('The system is not observable.');
    end
end
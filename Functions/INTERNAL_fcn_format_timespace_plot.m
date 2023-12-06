function INTERNAL_fcn_format_timespace_plot()
    % define figure properties
    opts.width      = 8.8;
    opts.height     = 6;
    % opts.fontType   = 'Times New Roman';
    % opts.fontSize   = 8;
    fig = gcf;
    % scaling
    fig.Units               = 'centimeters';
    fig.Position(3)         = opts.width;
    fig.Position(4)         = opts.height;

    % set text properties
    % set(fig.Children, ...
    %     'FontName',     'Times New Roman', ...
    %     'FontSize',     8);

    % remove unnecessary white space
    set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('t [s]')
    view([36 30])
end

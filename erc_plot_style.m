function erc_plot_style(fig)
% Explicit colors keep report graphics readable regardless of desktop theme.
set(fig,'Color','w');
set(findall(fig,'Type','axes'),'Color','w','XColor','k','YColor','k');
set(findall(fig,'Type','text'),'Color','k');
set(findall(fig,'Type','legend'),'TextColor','k','Color','w');
end

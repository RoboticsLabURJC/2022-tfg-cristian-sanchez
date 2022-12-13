#! /usr/bin/env python
import friss as fr
import matplotlib.pylab as plt
from matplotlib.widgets import Slider, CheckButtons

def update(val):
    global im_default, im_fixed, cbar_def, fixed_selected
    
    # Data extraction from sliders
    Pt = power_t.val
    Gt = gain_t.val
    Gr = gain_r.val
    Fq = freq.val * (10**9)
    l = losses_factor.val
    n = loss_exp.val

    # Recalculate data with the new values
    my_model.set_values(Pt, Gt, Gr, Fq, l, n)
    data = my_model.model_power_signal((20, 20))

    # Only update the one that is selected
    if fixed_selected:
        im_fixed.set_data(data)
    else:
        im_default.set_data(data)
        cbar_def.mappable.autoscale()

    # Terminal debugging
    my_model.test(2, 10)

def checkbox(label): 
    global fixed_selected
    fixed_selected = not fixed_selected

if __name__ ==  "__main__":
    # -- INIT DATA -- #
    my_model = fr.Friss()
    data = my_model.model_power_signal((20, 20))
    mesh_min, mesh_max = my_model.CUSTOM_RANGE

    # -- INIT FOR DISPLAY -- #
    fig = plt.figure("Heatmap")
    ax_def = fig.add_subplot(221)
    ax_fix = fig.add_subplot(222)
    ax_def.set_title("Default")
    title = "Fixed limits (" + str(mesh_min) + ", " + str(mesh_max) +")"
    ax_fix.set_title(title)

    # -- SHOW DATA & COLORBARS -- #
    im_default = ax_def.imshow(data, cmap = 'afmhot', aspect='equal', origin='lower')
    cbar_def = plt.colorbar(im_default, ax=ax_def)
    im_fixed = ax_fix.imshow(data, cmap = 'afmhot', aspect='equal', origin='lower', 
                          vmin=mesh_min, vmax=mesh_max)
    cbar_fix = plt.colorbar(im_fixed, ax=ax_fix)

    # -- SLIDERS -- #
    ax_Pt = fig.add_axes([0.15, 0.3, 0.65, 0.03])
    ax_Gt = fig.add_axes([0.15, 0.25, 0.65, 0.03])
    ax_Gr = fig.add_axes([0.15, 0.2, 0.65, 0.03])
    ax_Fq = fig.add_axes([0.15, 0.15, 0.65, 0.03])
    ax_l = fig.add_axes([0.15, 0.1, 0.65, 0.03])
    ax_n = fig.add_axes([0.15, 0.05, 0.65, 0.03]) 
    
    power_t = Slider(ax_Pt, 'Pt (W)', 0.01, 100.0, 1.0)
    gain_t = Slider(ax_Gt, 'Gt (W)', 0.01, 100.0, 1.0)
    gain_r = Slider(ax_Gr, 'Gr (W)', 0.01, 100.0, 1.0)
    freq = Slider(ax_Fq, 'Fq (GHz)', 0.01, 10.0, fr.FREQ_WIFI/(10**9))
    losses_factor = Slider(ax_l, 'L', 0.01, 10.0, 1.0)
    loss_exp = Slider(ax_n, 'n', 1.6, 6.0, 2.0)    

    power_t.on_changed(update)
    gain_t.on_changed(update)
    gain_r.on_changed(update)
    freq.on_changed(update)
    losses_factor.on_changed(update)
    loss_exp.on_changed(update)

    # -- CHECKBUTTON -- # 
    fixed_selected = False   
    button_ax = fig.add_axes([0.4, 0.37, 0.18, 0.1])
    check = CheckButtons(button_ax, ["Fixed limits"], [fixed_selected])
    check.on_clicked(checkbox)

    # Terminal debugging
    my_model.test(2, 10)
    
    # Display app
    plt.show()
    

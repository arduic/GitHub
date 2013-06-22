package net.bluetoothviewer;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.View;

public class DrawView extends View {
    Paint paint = new Paint();
    public int xdraw;
    public int ydraw;

    public DrawView(Context context,int x,int y) {
        super(context);
        xdraw=x;
        ydraw=y;
        paint.setColor(Color.WHITE);
    }

    @Override
    public void onDraw(Canvas canvas) {
            canvas.drawLine(450, 450, xdraw, ydraw, paint);
    }

}
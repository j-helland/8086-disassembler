bits 16

sub bx, [bx+si]
sub bx, [bp]

sub si, 2
sub bp, 2
sub cx, 8

sub bx, [bp + 0]
sub cx, [bx + 2]
sub bh, [bp + si + 4]
sub di, [bp + di + 6]

sub [bx+si], bx
sub [bp], bx

sub [bp + 0], bx
sub [bx + 2], cx
sub [bp + si + 4], bh
sub [bp + di + 6], di

sub byte [bx], 34
sub word [bx], 34
sub word [bx + di], 29

sub ax, [bp]
sub al, [bx + si]

sub ax, bx
sub al, ah

sub ax, 1000
sub al, -30
sub al, 9
function R = get_R(body,timestep,body_xi,body_yi,body_zi)
    xi = body_xi(body,timestep,:);
    yi = body_yi(body,timestep,:);
    zi = body_zi(body,timestep,:);
    R(:,1) = xi;
    R(:,2) = yi;
    R(:,3) = zi;
end
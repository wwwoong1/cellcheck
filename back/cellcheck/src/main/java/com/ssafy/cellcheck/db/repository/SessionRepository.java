package com.ssafy.cellcheck.db.repository;

import com.ssafy.cellcheck.db.entity.Session;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.stereotype.Repository;

@Repository
public interface SessionRepository  extends JpaRepository<Session, Long> {
    @Query("SELECT s FROM Session s ORDER BY s.createdAt DESC LIMIT 1")
    Session findLatestSession();
}
